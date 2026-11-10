"""Generate FreeMASTER 3.2 .pmpx files using the user's existing XML schema.

This helper only generates local XML. It does not change firmware or connection settings.
Public API: generate(output_path, *, title, elf_path, groups, template_path=None)
and validate(path).  See generate's docstring for the input format.
"""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET


DEFAULT_TEMPLATE = Path(__file__).resolve().with_name("样式模板.xml")
ITEM_TYPES = {"CPrjItemProject", "CPrjItemBlock", "CPrjItemScope"}
OBJECT_REF = re.compile(r"^ObjId\d+$")


def _get(obj, name, default=None):
    item = obj.find(f"member[@name='{name}']")
    return item.text if item is not None else default


def _set(obj, name, value, type_=None):
    item = obj.find(f"member[@name='{name}']")
    if item is None:
        if type_ is None:
            raise KeyError(f"No member {name} in {obj.get('type')}")
        item = ET.SubElement(obj, "member", {"name": name, "type": type_})
    item.text = "" if value is None else str(value)
    return item


def _addr(value):
    if value is None:
        return "0x0"
    return f"0x{int(value, 0) if isinstance(value, str) else int(value):X}"


class _Builder:
    def __init__(self, template):
        self.source = ET.parse(template).getroot()
        self.original = {o.get("name"): o for o in self.source.find("objects")}
        self.root = ET.Element("xmlarchive", dict(self.source.attrib))
        self.main = ET.SubElement(self.root, "main")
        self.objects = ET.SubElement(self.root, "objects")
        self.serial = 0
        self.styles = {}
        self.variables = {}
        self.specs = {}
        self.palette = ["0x2E2DED", "0xE47800", "0x2DA52D", "0xC040C0", "0xD0A000", "0x202020"]

    def new(self, type_, template=None):
        obj = deepcopy(template) if template is not None else ET.Element("object", {"type": type_})
        obj.set("name", f"ObjId{self.serial:04d}")
        obj.set("type", type_)
        obj.tail = None
        self.serial += 1
        self.objects.append(obj)
        return obj

    def sample(self, type_):
        return next(o for o in self.original.values() if o.get("type") == type_)

    def ref(self, obj):
        return obj.get("name")

    def list(self, values=(), type_="CObList"):
        obj = self.new(type_)
        for i, value in enumerate(values):
            _set(obj, f"item{i}", value, "CObject_ptr")
        return obj

    def copy_graph(self, source_id):
        """Clone style-only graphs; share them within a generated document."""
        if source_id in self.styles:
            return self.styles[source_id]
        old = self.original[source_id]
        obj = self.new(old.get("type"), old)
        target = self.ref(obj)
        self.styles[source_id] = target
        for item in obj.iter("member"):
            if item.text and OBJECT_REF.fullmatch(item.text):
                item.text = self.copy_graph(item.text)
        return target

    def register(self, variable):
        spec = dict(variable)
        expr = spec["expr"]
        for key, default in [("label", expr), ("unit", ""), ("writable", False),
                             ("min", None), ("max", None), ("description", ""),
                             ("last_addr", spec.get("address")), ("period", 100)]:
            spec.setdefault(key, default)
        if spec["byte_size"] not in (1, 2, 4, 8):
            raise ValueError(f"Unsupported byte_size for {expr}")
        if spec["treat_as"] not in (0, 1, 2):
            raise ValueError(f"Unsupported treat_as for {expr}")
        if spec["treat_as"] == 2 and spec["byte_size"] not in (4, 8):
            raise ValueError(f"Float must be 4/8 bytes: {expr}")
        if expr in self.variables:
            previous = self.specs[expr]
            for key in ("byte_size", "treat_as", "writable", "min", "max", "last_addr"):
                if previous[key] != spec[key]:
                    raise ValueError(f"Inconsistent {key} for repeated variable {expr}")
            return self.variables[expr]
        old = self.sample("CVariable")
        obj = self.new("CVariable", old)
        replacements = {
            "name": expr, "address": _addr(spec["last_addr"]), "last_addr": _addr(spec["last_addr"]),
            "byte_size": spec["byte_size"], "treat_as": spec["treat_as"],
            "unit": spec["unit"], "period": spec["period"],
            "description": spec["description"], "comment": "",
            "type_fmt": "IEEE floating point" if spec["treat_as"] == 2 else "Fixed point number",
            "modif_mode": int(spec["writable"]),
            "modif_edit_mode": 2,  # Match the user's known-working FreeMASTER editing configuration.
            "show_as": 4 if spec["treat_as"] == 2 else 0,  # Floating decimal display, matching the known-working project.
            "show_num_afp_digs": 1 if spec["treat_as"] == 2 else 0,
            "modif_set_min": "" if spec["min"] is None else spec["min"],
            "modif_set_max": "" if spec["max"] is None else spec["max"],
            "modif_set_step": "", "modif_address": "", "modif_last_addr": "0x0",
            "enum_enabled": 0, "enum": "", "value_trf": "NULL",
        }
        # FreeMASTER user guide, Modifying tab: mode 0 is read-only; mode 1
        # accepts any value within modif_set_min/max. Firmware still validates
        # complete requests, including requests arriving from another debugger.
        for name, value in replacements.items():
            _set(obj, name, value)
        self.variables[expr] = self.ref(obj)
        self.specs[expr] = spec
        return self.ref(obj)

    def block(self, title, parent_id, expressions, *, root=False):
        type_ = "CPrjItemProject" if root else "CPrjItemBlock"
        old = self.sample(type_)
        obj = self.new(type_, old)
        _set(obj, "name", title)
        _set(obj, "parent_item", parent_id)
        _set(obj, "tree_expand", 1)
        _set(obj, "href", "")
        _set(obj, "data_capture_dir", "")
        _set(obj, "watch_variables", self.ref(self.list(self.variables[e] for e in expressions)))
        _set(obj, "watch_functions", self.ref(self.list()))
        children = self.list()
        _set(obj, "child_items", self.ref(children))
        for member in ("cell_format", "column_info"):
            _set(obj, member, self.copy_graph(_get(old, member)))
        rows = self.new("CMapObToObEx")
        for i, expr in enumerate(expressions):
            info = self.new("CPrjItemBlock_VarInfo", self.sample("CPrjItemBlock_VarInfo"))
            formats = self.new("CObArray")
            formats.set("length", "0")
            _set(info, "row_cell_formats", self.ref(formats))
            _set(rows, f"key{i}", self.variables[expr], "CObject_ptr")
            _set(rows, f"value{i}", self.ref(info), "CObject_ptr")
        _set(obj, "variable_info", self.ref(rows))
        return obj, children

    def scope(self, spec, parent_id):
        expressions = spec.get("exprs", spec.get("expressions"))
        if expressions is None:
            raise ValueError("Scope requires exprs or expressions")
        if not expressions:
            raise ValueError("Scope needs at least one expression")
        for expr in expressions:
            if expr not in self.variables:
                raise ValueError(f"Scope variable not declared in a group: {expr}")
        old = self.sample("CPrjItemScope")
        obj = self.new("CPrjItemScope", old)
        # Remove the template's three curve and six axis references.
        for item in list(obj):
            if re.fullmatch(r"var\d+_info|yblock\d+_info", item.get("name", "")):
                obj.remove(item)
        _set(obj, "name", spec["title"])
        _set(obj, "parent_item", parent_id)
        _set(obj, "href", "")
        _set(obj, "data_capture_dir", "")
        _set(obj, "graph_var_count", len(expressions))
        _set(obj, "var_count", len(expressions))
        _set(obj, "scope_period", f"{float(spec.get('period', .02)):.20e}")
        _set(obj, "scope_x_width", f"{float(spec.get('width', 5)):.20e}")
        units = list(dict.fromkeys(self.specs[e]["unit"] for e in expressions))
        if len(units) > 6:
            raise ValueError("The template supports at most six independent Y axis blocks")
        for i, expr in enumerate(expressions):
            curve = self.new("CPrjItemRS_VarInfo", self.sample("CPrjItemRS_VarInfo"))
            _set(curve, "variable", self.variables[expr])
            _set(curve, "visible", 1)
            _set(curve, "y_block", units.index(self.specs[expr]["unit"]))
            _set(curve, "color", self.palette[i % len(self.palette)])
            _set(obj, f"var{i}_info", self.ref(curve), "CPrjItemRS_VarInfo")
        for i in range(6):
            axis = self.new("CPrjItemRS_YBlockInfo", self.sample("CPrjItemRS_YBlockInfo"))
            _set(axis, "join_class", i)
            _set(axis, "laxis_label", units[i] if i < len(units) else "")
            _set(axis, "raxis_label", "")
            _set(obj, f"yblock{i}_info", self.ref(axis), "CPrjItemRS_YBlockInfo")
        return obj


def generate(output_path, *, title, elf_path, groups, template_path=None, root_expressions=()):
    """Generate a .pmpx and return validate()'s report.

    groups = [{"title": "第1批 中文组", "variables": [
        {"expr": "g_chassis_status.ready", "label": "底盘就绪", "byte_size": 1,
         "treat_as": 0, "writable": False, "unit": "", "min": None, "max": None,
         "last_addr": 0x20000000, "description": "只读状态"}],
        "scopes": [{"title": "轮速曲线", "exprs": ["..."]}]}]

    Variables are shared by expression; inconsistent type/permissions are rejected.
    All source symbols/addresses must already have been checked against elf_path.
    This helper validates XML structure, not a live FreeMASTER connection.
    """
    b = _Builder(template_path or DEFAULT_TEMPLATE)
    for group in groups:
        for variable in group.get("variables", []) + group.get("scope_variables", []):
            b.register(variable)
    old_doc = b.sample("CPrjDoc")
    doc = b.new("CPrjDoc", old_doc)
    _set(b.main, "document", b.ref(doc), "CPrjDoc")
    enums = b.new("CEnumList", b.sample("CEnumList"))
    _set(enums, "enums_list", b.ref(b.list()))
    _set(doc, "enum_db_man", b.ref(enums))
    _set(doc, "variables", b.ref(b.list(b.variables.values())))
    for key in ("stimulators", "functions"):
        _set(doc, key, b.ref(b.list()))
    _set(doc, "function_msgs", b.ref(b.new("CMapWordToOb")))
    map_types = []
    for type_ in ("CPrjDoc_ElfFileType", "CPrjDoc_Hi509FileType"):
        map_types.append(b.ref(b.new(type_, b.sample(type_))))
    _set(doc, "mapfile_types", b.ref(b.list(map_types)))
    info = b.new("CPrjDoc_MapFileInfo", b.sample("CPrjDoc_MapFileInfo"))
    _set(info, "file_name", str(elf_path))
    _set(info, "file_type", map_types[0])
    _set(doc, "mapfile_list", b.ref(b.list([b.ref(info)])))
    _set(doc, "mapfile_current", 0)
    _set(doc, "html_auth_xobjects", b.ref(b.new("CStringList")))
    _set(doc, "html_control_page", "")
    _set(doc, "tmpdir_orig", "")
    _set(doc, "demo_enable", 0)
    _set(doc, "demo_protect", 0)
    _set(doc, "demo_password", "")
    _set(doc, "have_packed_files", 0)
    _set(doc, "have_layout_data", 0)
    # Retain the schema's required dialog object without inheriting a target
    # flash file, write buffers or programming addresses. Generation never
    # connects to a board or invokes the flash programmer.
    flash = b.new("CFlashProgDlg", b.sample("CFlashProgDlg"))
    for key in ("csw_addr", "buff_addr", "file_name"):
        _set(flash, key, "")
    for key in ("csw_size", "buff_size", "ex_addr", "erase_before_write",
                "verify_after_write", "block_safe_write"):
        _set(flash, key, 0)
    for key in ("mem_offset", "filt_start", "filt_size"):
        _set(flash, key, "0x0")
    _set(doc, "flash_prog", b.ref(flash))
    root, children = b.block(title, "NULL", root_expressions, root=True)
    _set(doc, "project_root", b.ref(root))
    for index, group in enumerate(groups):
        block, grandchildren = b.block(group["title"], b.ref(root),
                                       [v["expr"] for v in group.get("variables", [])])
        _set(children, f"item{index}", b.ref(block), "CObject_ptr")
        for scope_index, scope in enumerate(group.get("scopes", [])):
            graph = b.scope(scope, b.ref(block))
            _set(grandchildren, f"item{scope_index}", b.ref(graph), "CObject_ptr")
    ET.indent(b.root, space="\t")
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n' +
                      ET.tostring(b.root, encoding="unicode") + "\n", encoding="utf-8", newline="\n")
    return validate(output)


def update_groups(path, groups):
    """Trim existing watches/curves without replacing connection or display settings.

    Group titles identify existing blocks. Existing definitions and row/curve
    styles are reused; new variables require a fully bound address/type spec.
    Scopes are matched by title, then position, and never created. Validation
    completes in a temporary file before the original is atomically replaced.
    """
    path = Path(path)
    root = ET.parse(path).getroot()
    objects = root.find("objects")
    serial = max(int(o.get("name")[5:]) for o in objects
                 if OBJECT_REF.fullmatch(o.get("name", ""))) + 1
    document_id = _get(root.find("main"), "document")
    documents = [o for o in objects if o.get("name") == document_id and
                 o.get("type") == "CPrjDoc"]
    if len(documents) != 1:
        raise ValueError("Expected one unambiguous CPrjDoc")
    if sum(o.get("name") == document_id for o in objects) > 1:
        # FreeMASTER's native save can number the document and first variable
        # alike. Only the explicitly typed document reference denotes CPrjDoc;
        # generic watch/list references continue to denote the first variable.
        new_id = f"ObjId{serial:04d}"
        serial += 1
        documents[0].set("name", new_id)
        for member in root.iter("member"):
            if member.text == document_id and member.get("type") == "CPrjDoc":
                member.text = new_id

    def signature(obj):
        return (tuple(sorted(obj.attrib.items())), (obj.text or "").strip(),
                tuple(signature(child) for child in obj))

    unique = {}
    for obj in list(objects):
        name = obj.get("name")
        if name in unique:
            if signature(unique[name]) != signature(obj):
                raise ValueError(f"Ambiguous duplicate object: {name}")
            objects.remove(obj)  # Identical repeated native-save style objects.
        else:
            unique[name] = obj
    byid = {o.get("name"): o for o in objects}
    document = byid[_get(root.find("main"), "document")]
    variable_list = byid[_get(document, "variables")]
    variables = {_get(byid[m.text], "name"): m.text for m in variable_list}
    all_variable_ids = [m.text for m in variable_list]

    def title_key(text):
        # Native FreeMASTER save serializes CP936 bytes as Latin-1 entities.
        # Decode only for title matching; connection/ELF strings stay intact.
        try:
            decoded = text.encode("latin1").decode("gbk")
            return decoded if any("\u4e00" <= c <= "\u9fff" for c in decoded) else text
        except UnicodeError:
            return text

    blocks = {title_key(_get(o, "name")): o for o in objects
              if o.get("type") == "CPrjItemBlock"}

    for group in groups:
        for spec in group.get("variables", []) + group.get("scope_variables", []):
            expr = spec["expr"]
            if expr in variables:
                continue
            if not {"last_addr", "byte_size", "treat_as"}.issubset(spec):
                raise ValueError(f"New variable requires an ELF-bound spec: {expr}")
            if spec["byte_size"] not in (1, 2, 4, 8) or spec["treat_as"] not in (0, 1, 2):
                raise ValueError(f"Invalid variable type: {expr}")
            if spec["treat_as"] == 2 and spec["byte_size"] not in (4, 8):
                raise ValueError(f"Invalid floating variable width: {expr}")
            obj = deepcopy(byid[all_variable_ids[0]])
            variable_id = f"ObjId{serial:04d}"
            serial += 1
            obj.set("name", variable_id)
            replacements = {
                "name": expr, "address": _addr(spec["last_addr"]), "last_addr": _addr(spec["last_addr"]),
                "byte_size": spec["byte_size"], "treat_as": spec["treat_as"],
                "unit": spec.get("unit", ""), "period": spec.get("period", 100),
                "description": spec.get("description", ""), "comment": "",
                "type_fmt": "IEEE floating point" if spec["treat_as"] == 2 else "Fixed point number",
                "modif_mode": int(spec.get("writable", False)),
                "modif_set_min": "" if spec.get("min") is None else spec["min"],
                "modif_set_max": "" if spec.get("max") is None else spec["max"],
                "modif_set_step": "", "modif_address": "", "modif_last_addr": "0x0",
                "enum_enabled": 0, "enum": "", "value_trf": "NULL",
            }
            for name, value in replacements.items():
                _set(obj, name, value)
            objects.append(obj)
            byid[variable_id] = obj
            variables[expr] = variable_id
            all_variable_ids.append(variable_id)

    def replace_list(obj, references):
        obj[:] = []
        for i, reference in enumerate(references):
            _set(obj, f"item{i}", reference, "CObject_ptr")

    for spec in groups:
        if spec["title"] not in blocks:
            raise ValueError(f"Existing group not found: {spec['title']}")
        block = blocks[spec["title"]]
        _set(block, "name", spec["title"])
        watches = byid[_get(block, "watch_variables")]
        rows = byid[_get(block, "variable_info")]
        old_rows = {_get(rows, f"key{i}"): _get(rows, f"value{i}")
                    for i in range(len(watches))}
        row_template = byid[next(iter(old_rows.values()))] if old_rows else None
        requested = list(dict.fromkeys(v["expr"] for v in spec.get("variables", [])))
        unknown = [expr for expr in requested if expr not in variables]
        if unknown:
            raise ValueError(f"Variables absent from existing project: {unknown}")
        replace_list(watches, [variables[expr] for expr in requested])
        rows[:] = []
        for i, expr in enumerate(requested):
            variable_id = variables[expr]
            row_id = old_rows.get(variable_id)
            if row_id is None:
                if row_template is None:
                    raise ValueError(f"No existing row style for {spec['title']}")
                row = deepcopy(row_template)
                row_id = f"ObjId{serial:04d}"
                serial += 1
                row.set("name", row_id)
                objects.append(row)
                byid[row_id] = row
            _set(rows, f"key{i}", variable_id, "CObject_ptr")
            _set(rows, f"value{i}", row_id, "CObject_ptr")

        children = byid[_get(block, "child_items")]
        scopes = spec.get("scopes", [])
        kept_children = []
        scope_index = 0
        for member in list(children):
            child = byid[member.text]
            if child.get("type") != "CPrjItemScope":
                kept_children.append(member.text)
                continue
            desired = next((s for s in scopes if s["title"] == title_key(_get(child, "name"))), None)
            if desired is None and scope_index < len(scopes):
                desired = scopes[scope_index]
            scope_index += 1
            if desired is None:
                continue
            expressions = desired.get("exprs", desired.get("expressions", []))
            if not expressions:
                continue
            curves = {}
            for i in range(int(_get(child, "var_count"))):
                curve_id = _get(child, f"var{i}_info")
                curves[_get(byid[_get(byid[curve_id], "variable")], "name")] = curve_id
            missing = [expr for expr in expressions if expr not in curves]
            if missing:
                raise ValueError(f"Curves absent from existing scope: {missing}")
            for member in list(child):
                if re.fullmatch(r"var\d+_info", member.get("name", "")):
                    child.remove(member)
            for i, expr in enumerate(expressions):
                _set(child, f"var{i}_info", curves[expr], "CPrjItemRS_VarInfo")
            _set(child, "var_count", len(expressions))
            _set(child, "graph_var_count", len(expressions))
            _set(child, "name", desired["title"])
            kept_children.append(child.get("name"))
        replace_list(children, kept_children)

    used_variables = set()
    for obj in objects:
        if obj.get("type") in ("CPrjItemBlock", "CPrjItemProject"):
            used_variables.update(m.text for m in byid[_get(obj, "watch_variables")])
        elif obj.get("type") == "CPrjItemScope":
            # Detached scopes are removed below; keeping their variables here
            # would leave unused entries in the global variable list.
            parent = byid[_get(obj, "parent_item")]
            if obj.get("name") in {m.text for m in byid[_get(parent, "child_items")]}:
                for i in range(int(_get(obj, "var_count"))):
                    used_variables.add(_get(byid[_get(obj, f"var{i}_info")], "variable"))
    replace_list(variable_list, [reference for reference in all_variable_ids
                                if reference in used_variables])
    reachable = set()

    def mark(reference):
        if reference in reachable:
            return
        reachable.add(reference)
        for member in byid[reference].iter("member"):
            if member.text and OBJECT_REF.fullmatch(member.text):
                mark(member.text)

    mark(_get(root.find("main"), "document"))
    objects[:] = [obj for obj in objects if obj.get("name") in reachable]
    ET.indent(root, space="\t")
    payload = ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n' +
               ET.tostring(root, encoding="unicode") + "\n").encode("utf-8")
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=path.name + ".", suffix=".tmp",
                                         delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
        result = validate(temporary)
        temporary.replace(path)
        return result
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def validate(path):
    """Check references, watches/row maps, tree parents, scopes and UTF-8 XML."""
    root = ET.parse(path).getroot()
    assert root.tag == "xmlarchive"
    objects = list(root.find("objects"))
    byid = {o.get("name"): o for o in objects}
    assert len(byid) == len(objects), "Duplicate ObjId"
    for item in root.iter("member"):
        if item.text and OBJECT_REF.fullmatch(item.text):
            assert item.text in byid, f"Missing reference {item.text}"
    reachable = set()

    def mark(id_):
        if id_ in reachable:
            return
        reachable.add(id_)
        for member in byid[id_].iter("member"):
            if member.text and OBJECT_REF.fullmatch(member.text):
                mark(member.text)

    mark(_get(root.find("main"), "document"))
    assert reachable == set(byid), "Unreferenced template objects retained"
    doc = byid[_get(root.find("main"), "document")]
    project = byid[_get(doc, "project_root")]
    assert project.get("type") == "CPrjItemProject"
    assert _get(project, "parent_item") == "NULL"
    variable_ids = {_get_member.text for _get_member in byid[_get(doc, "variables")]}
    assert all(byid[i].get("type") == "CVariable" for i in variable_ids)
    for id_ in variable_ids:
        assert _get(byid[id_], "modif_mode") in ("0", "1"), "Unexpected modification mode"
    map_list = byid[_get(doc, "mapfile_list")]
    assert len(map_list) == 1, "Expected one selected ELF"
    tree_ids = set()

    def visit(obj):
        id_ = obj.get("name")
        assert id_ not in tree_ids, "Tree cycle or duplicated child"
        tree_ids.add(id_)
        if obj.get("type") == "CPrjItemScope":
            count = int(_get(obj, "var_count"))
            assert int(_get(obj, "graph_var_count")) == count
            curves = [m for m in obj if re.fullmatch(r"var\d+_info", m.get("name", ""))]
            assert len(curves) == count
            for i in range(count):
                curve = byid[_get(obj, f"var{i}_info")]
                assert curve.get("type") == "CPrjItemRS_VarInfo"
                assert _get(curve, "variable") in variable_ids
                assert 0 <= int(_get(curve, "y_block")) < 6
            for i in range(6):
                assert byid[_get(obj, f"yblock{i}_info")].get("type") == "CPrjItemRS_YBlockInfo"
            return
        watches = [m.text for m in byid[_get(obj, "watch_variables")]]
        rows = byid[_get(obj, "variable_info")]
        keys = [_get(rows, f"key{i}") for i in range(len(watches))]
        assert keys == watches, "Watch list / row style mismatch"
        assert len(rows) == 2 * len(watches)
        assert set(watches).issubset(variable_ids)
        for i in range(len(watches)):
            assert byid[_get(rows, f"value{i}")].get("type") == "CPrjItemBlock_VarInfo"
        for member in byid[_get(obj, "child_items")]:
            child = byid[member.text]
            assert child.get("type") in ITEM_TYPES
            assert _get(child, "parent_item") == id_, "Parent/child mismatch"
            visit(child)

    visit(project)
    all_tree_ids = {o.get("name") for o in objects if o.get("type") in ITEM_TYPES}
    assert tree_ids == all_tree_ids, "Unattached old tree items"
    assert not list(byid[_get(doc, "functions")]), "Unrelated functions retained"
    assert not list(byid[_get(doc, "stimulators")]), "Unrelated stimulators retained"
    flash = byid[_get(doc, "flash_prog")]
    for key in ("csw_addr", "buff_addr", "file_name"):
        assert not _get(flash, key), "Unexpected flash programming configuration"
    text = Path(path).read_text(encoding="utf-8")
    assert "\ufffd" not in text, "Replacement character in XML"
    return {"objects": len(objects), "variables": len(variable_ids),
            "groups": sum(o.get("type") == "CPrjItemBlock" for o in objects),
            "scopes": sum(o.get("type") == "CPrjItemScope" for o in objects),
            "elf": _get(byid[next(iter(map_list)).text], "file_name"),
            "title": _get(project, "name")}
