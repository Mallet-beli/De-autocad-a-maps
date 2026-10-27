import math
import zipfile
import traceback
from pathlib import Path
from collections import defaultdict, Counter
import heapq

# =========================
# CONFIG
# =========================

GOOGLE_MAPS_ZOOM = 17
POINT_TOLERANCE = 1.0
ANNOTATION_DISTANCE = 12.0
SAMPLE_POINTS = 80

# Límites amplios de Paraguay y alrededores.
# No se usa como verdad absoluta, solo para puntuar transformaciones plausibles.
REGION_BOUNDS = {
    "lat_min": -30.5,
    "lat_max": -18.0,
    "lon_min": -65.5,
    "lon_max": -52.0,
}

# CRS UTM sur razonables para Paraguay y alrededores.
CANDIDATE_EPSG = [32719, 32720, 32721, 32722]

BAD_LAYER_WORDS = {
    "rail", "railway", "railways",
    "road", "roads", "street", "streets", "calle", "calles", "avenida",
    "text", "label", "labels", "name", "names", "nombres",
    "parcel", "parcels", "lot", "lots", "building", "buildings",
    "river", "rivers", "water", "boundary", "boundaries", "border",
    "contour", "topo", "topography", "manzana", "catastro",
    "eje", "centerline", "grid", "lote", "muro", "muros"
}

GOOD_LAYER_WORDS = {
    "ruta", "route", "recorrido", "track", "path", "camino", "trayecto",
    "fibra", "cable", "ducto", "linea", "línea"
}

# Colores destacados frecuentes. Se usan como ayuda, no como requisito.
HIGHLIGHT_ACI = {1, 10, 20, 30, 40, 210}


# =========================
# LOG
# =========================

def log(msg):
    print(msg)
    with open("debug_ruta.log", "a", encoding="utf-8") as f:
        f.write(str(msg) + "\n")


# =========================
# UTILIDADES GEOMÉTRICAS
# =========================

def distancia(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def longitud(puntos):
    total = 0.0
    for i in range(1, len(puntos)):
        total += distancia(puntos[i - 1], puntos[i])
    return total

def bbox_diagonal(puntos):
    if not puntos:
        return 0.0
    xs = [p[0] for p in puntos]
    ys = [p[1] for p in puntos]
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))

def bbox_center(puntos):
    xs = [p[0] for p in puntos]
    ys = [p[1] for p in puntos]
    return ((min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0)

def snap_key(p):
    return (round(p[0] / POINT_TOLERANCE), round(p[1] / POINT_TOLERANCE))

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def point_to_segment_distance(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p

    dx = bx - ax
    dy = by - ay
    if dx == 0 and dy == 0:
        return distancia(p, a)

    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = clamp(t, 0.0, 1.0)

    qx = ax + t * dx
    qy = ay + t * dy
    return math.hypot(px - qx, py - qy)

def point_to_polyline_distance(p, poly):
    if len(poly) < 2:
        return float("inf")
    best = float("inf")
    for i in range(1, len(poly)):
        d = point_to_segment_distance(p, poly[i - 1], poly[i])
        if d < best:
            best = d
    return best

def polyline_sample_points(poly, n=SAMPLE_POINTS):
    if len(poly) < 2:
        return poly[:]

    seg_lengths = []
    total = 0.0
    for i in range(1, len(poly)):
        l = distancia(poly[i - 1], poly[i])
        seg_lengths.append(l)
        total += l

    if total == 0:
        return [poly[0]]

    targets = [total * i / max(1, n - 1) for i in range(n)]
    samples = []
    seg_i = 0
    acc = 0.0

    for t in targets:
        while seg_i < len(seg_lengths) - 1 and acc + seg_lengths[seg_i] < t:
            acc += seg_lengths[seg_i]
            seg_i += 1

        a = poly[seg_i]
        b = poly[seg_i + 1]
        seg_len = seg_lengths[seg_i]

        if seg_len == 0:
            samples.append(a)
            continue

        local_t = (t - acc) / seg_len
        x = a[0] + (b[0] - a[0]) * local_t
        y = a[1] + (b[1] - a[1]) * local_t
        samples.append((x, y))

    return samples

def coverage_ratio(poly, annotation_points, distance_threshold):
    if not annotation_points or len(poly) < 2:
        return 0.0

    samples = polyline_sample_points(poly, SAMPLE_POINTS)
    covered = 0

    for s in samples:
        for ap in annotation_points:
            if distancia(s, ap) <= distance_threshold:
                covered += 1
                break

    return covered / max(1, len(samples))


# =========================
# CAD
# =========================

def cargar_ezdxf():
    import ezdxf
    return ezdxf

def abrir_cad(path_archivo, ezdxf):
    ext = path_archivo.suffix.lower()

    if ext == ".dxf":
        return ezdxf.readfile(str(path_archivo))

    if ext == ".dwg":
        from ezdxf.addons import odafc
        return odafc.readfile(str(path_archivo))

    raise RuntimeError(f"Formato no soportado: {ext}")

def get_entity_color_aci(e):
    try:
        return int(getattr(e.dxf, "color", 256))
    except Exception:
        return 256

def is_highlight_color(e):
    return get_entity_color_aci(e) in HIGHLIGHT_ACI

def layer_score(layer_name):
    name = (layer_name or "").lower()
    score = 0.0

    for w in BAD_LAYER_WORDS:
        if w in name:
            score -= 180.0

    for w in GOOD_LAYER_WORDS:
        if w in name:
            score += 220.0

    return score

def entity_style_score(e):
    score = 0.0

    if is_highlight_color(e):
        score += 180.0

    try:
        color = int(getattr(e.dxf, "color", 256))
        if color not in (0, 7, 8, 9, 256):
            score += 40.0
    except Exception:
        pass

    try:
        lw = int(getattr(e.dxf, "lineweight", -1))
        if lw > 0:
            score += 50.0
    except Exception:
        pass

    try:
        lt = str(getattr(e.dxf, "linetype", "")).lower()
        if lt and lt not in ("bylayer", "continuous"):
            score += 20.0
    except Exception:
        pass

    return score

def extraer_puntos_entidad(e):
    t = e.dxftype()

    if t == "LWPOLYLINE":
        return [(p[0], p[1]) for p in e.get_points()]

    if t == "POLYLINE":
        pts = []
        for v in e.vertices:
            loc = v.dxf.location
            pts.append((loc.x, loc.y))
        return pts

    if t == "LINE":
        return [(e.dxf.start.x, e.dxf.start.y), (e.dxf.end.x, e.dxf.end.y)]

    if t == "SPLINE":
        try:
            pts = []
            for p in e.flattening(distance=0.5):
                pts.append((p[0], p[1]))
            return pts
        except Exception:
            return []

    return []

def entity_anchor_point(e):
    t = e.dxftype()

    try:
        if t == "TEXT":
            return (e.dxf.insert.x, e.dxf.insert.y)
        if t == "MTEXT":
            return (e.dxf.insert.x, e.dxf.insert.y)
        if t == "INSERT":
            return (e.dxf.insert.x, e.dxf.insert.y)
        if t == "POINT":
            return (e.dxf.location.x, e.dxf.location.y)
        if t == "DIMENSION":
            return (e.dxf.text_midpoint.x, e.dxf.text_midpoint.y)
    except Exception:
        pass

    return None

def text_weight(text):
    if not text:
        return 1.0

    t = str(text).strip()
    if not t:
        return 1.0

    score = 1.0
    digit_count = sum(ch.isdigit() for ch in t)

    if digit_count >= 3:
        score += 2.0
    if "," in t or "." in t:
        score += 0.5
    if len(t) >= 6:
        score += 0.5

    return score

def extract_annotations(doc):
    msp = doc.modelspace()
    annotations = []

    for e in msp:
        t = e.dxftype()
        if t not in ("TEXT", "MTEXT", "INSERT", "DIMENSION", "POINT"):
            continue

        pt = entity_anchor_point(e)
        if pt is None:
            continue

        raw_text = ""
        if t == "TEXT":
            raw_text = getattr(e.dxf, "text", "")
        elif t == "MTEXT":
            try:
                raw_text = e.text
            except Exception:
                raw_text = ""
        elif t == "INSERT":
            raw_text = getattr(e.dxf, "name", "")
        elif t == "DIMENSION":
            raw_text = "DIMENSION"
        elif t == "POINT":
            raw_text = "POINT"

        annotations.append({
            "type": t,
            "point": pt,
            "weight": text_weight(raw_text),
            "text": raw_text,
        })

    log(f"Anotaciones detectadas: {len(annotations)}")
    return annotations

def is_closed_polyline(e, puntos):
    try:
        if e.dxftype() == "LWPOLYLINE":
            return bool(e.closed)
        if e.dxftype() == "POLYLINE":
            return bool(e.is_closed)
    except Exception:
        pass
    return len(puntos) >= 3 and distancia(puntos[0], puntos[-1]) <= POINT_TOLERANCE


# =========================
# GEODATA / CRS
# =========================

def try_geodata_transform(doc, puntos_xy):
    """
    Intenta usar GEODATA si está disponible y produce coordenadas plausibles.
    No se considera obligatorio.
    """
    try:
        msp = doc.modelspace()
        geodata = msp.get_geodata()
        if not geodata:
            return None, None

        # Intento conservador: usar design/reference como traslación local.
        design = geodata.dxf.design_point
        ref = geodata.dxf.reference_point

        lon0 = ref[0]
        lat0 = ref[1]

        if not (-180 <= lon0 <= 180 and -90 <= lat0 <= 90):
            return None, None

        cos_lat = math.cos(math.radians(lat0))
        if abs(cos_lat) < 1e-9:
            return None, None

        puntos_geo = []
        for x, y in puntos_xy:
            dx = x - design[0]
            dy = y - design[1]
            lat = lat0 + dy / 111111.0
            lon = lon0 + dx / (111111.0 * cos_lat)
            puntos_geo.append((lat, lon))

        if transformation_is_plausible(puntos_geo):
            log("GEODATA utilizable detectado y aceptado.")
            return puntos_geo, "GEODATA_APPROX"

    except Exception as e:
        log(f"GEODATA no utilizable: {e}")

    return None, None


# =========================
# GRAFO PARA LINE
# =========================

def build_line_graph(line_entities):
    graph = defaultdict(list)
    edge_meta = []

    for idx, e in enumerate(line_entities):
        pts = extraer_puntos_entidad(e)
        if len(pts) != 2:
            continue

        a = snap_key(pts[0])
        b = snap_key(pts[1])
        if a == b:
            continue

        layer = getattr(e.dxf, "layer", "") or ""
        seg_len = distancia(pts[0], pts[1])
        style = entity_style_score(e)

        meta = {
            "idx": idx,
            "a": a,
            "b": b,
            "a_xy": pts[0],
            "b_xy": pts[1],
            "layer": layer,
            "length": seg_len,
            "style_score": style,
        }
        edge_meta.append(meta)

        graph[a].append((b, seg_len, len(edge_meta) - 1))
        graph[b].append((a, seg_len, len(edge_meta) - 1))

    return graph, edge_meta

def connected_components(graph):
    seen = set()
    comps = []

    for start in graph:
        if start in seen:
            continue

        stack = [start]
        seen.add(start)
        nodes = []

        while stack:
            u = stack.pop()
            nodes.append(u)
            for v, _, _ in graph[u]:
                if v not in seen:
                    seen.add(v)
                    stack.append(v)

        comps.append(nodes)

    return comps

def dijkstra_farthest(graph, start, allowed_nodes):
    dist = {start: 0.0}
    prev = {}
    pq = [(0.0, start)]
    allowed = set(allowed_nodes)

    while pq:
        d, u = heapq.heappop(pq)
        if d != dist[u]:
            continue

        for v, w, _ in graph[u]:
            if v not in allowed:
                continue

            nd = d + w
            if v not in dist or nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    farthest = max(dist, key=lambda k: dist[k]) if dist else start
    return farthest, dist, prev

def shortest_path_nodes(prev, end):
    path = [end]
    while path[-1] in prev:
        path.append(prev[path[-1]])
    path.reverse()
    return path

def node_to_xy(node):
    return (node[0] * POINT_TOLERANCE, node[1] * POINT_TOLERANCE)

def component_stats(graph, nodes):
    degrees = [len(graph[n]) for n in nodes]
    endpoints = sum(1 for d in degrees if d == 1)
    branch_nodes = sum(1 for d in degrees if d >= 3)
    return endpoints, branch_nodes

def extract_best_path_from_component(graph, nodes):
    if not nodes:
        return []

    endpoints = [n for n in nodes if len(graph[n]) == 1]
    seed = endpoints[0] if endpoints else nodes[0]

    a, _, _ = dijkstra_farthest(graph, seed, nodes)
    b, _, prev = dijkstra_farthest(graph, a, nodes)
    path_nodes = shortest_path_nodes(prev, b)

    return [node_to_xy(n) for n in path_nodes]

def dominant_layer_for_component(graph, nodes, edge_meta):
    node_set = set(nodes)
    visited_edges = set()
    counter = Counter()

    for u in nodes:
        for v, _, edge_idx in graph[u]:
            if v not in node_set or edge_idx in visited_edges:
                continue
            visited_edges.add(edge_idx)
            counter[edge_meta[edge_idx]["layer"]] += edge_meta[edge_idx]["length"]

    if not counter:
        return "AUTO_LINE_GRAPH"
    return counter.most_common(1)[0][0]

def style_score_for_component(graph, nodes, edge_meta):
    node_set = set(nodes)
    visited_edges = set()
    total = 0.0
    count = 0

    for u in nodes:
        for v, _, edge_idx in graph[u]:
            if v not in node_set or edge_idx in visited_edges:
                continue
            visited_edges.add(edge_idx)
            total += edge_meta[edge_idx]["style_score"]
            count += 1

    return total / count if count else 0.0


# =========================
# CANDIDATAS DE RUTA
# =========================

def annotation_features_for_polyline(poly, annotations):
    if len(poly) < 2:
        return {
            "near_count": 0,
            "near_weight_sum": 0.0,
            "coverage": 0.0,
            "weighted_coverage": 0.0,
        }

    diag = bbox_diagonal(poly)
    dynamic_dist = max(ANNOTATION_DISTANCE, min(40.0, diag * 0.03))

    near_count = 0
    near_weight_sum = 0.0
    near_points = []

    for ann in annotations:
        d = point_to_polyline_distance(ann["point"], poly)
        if d <= dynamic_dist:
            near_count += 1
            near_weight_sum += ann["weight"]
            near_points.append(ann["point"])

    coverage = coverage_ratio(poly, near_points, dynamic_dist)
    weighted_coverage = coverage * (1.0 + min(near_weight_sum / 20.0, 5.0))

    return {
        "near_count": near_count,
        "near_weight_sum": near_weight_sum,
        "coverage": coverage,
        "weighted_coverage": weighted_coverage,
    }

def recolectar_candidatas(doc, annotations):
    msp = doc.modelspace()
    candidatas = []
    line_entities = []

    for e in msp:
        tipo = e.dxftype()
        if tipo not in ("LWPOLYLINE", "POLYLINE", "LINE", "SPLINE"):
            continue

        puntos = extraer_puntos_entidad(e)
        if len(puntos) < 2:
            continue

        capa = getattr(e.dxf, "layer", "") or ""

        if tipo == "LINE":
            line_entities.append(e)
            continue

        ann = annotation_features_for_polyline(puntos, annotations)

        candidatas.append({
            "tipo": tipo,
            "capa": capa,
            "puntos": puntos,
            "longitud": longitud(puntos),
            "diag": bbox_diagonal(puntos),
            "cerrada": is_closed_polyline(e, puntos),
            "branch_nodes": 0,
            "endpoints": 2 if not is_closed_polyline(e, puntos) else 0,
            "style_score": entity_style_score(e),
            "near_count": ann["near_count"],
            "near_weight_sum": ann["near_weight_sum"],
            "coverage": ann["coverage"],
            "weighted_coverage": ann["weighted_coverage"],
        })

    graph, edge_meta = build_line_graph(line_entities)
    comps = connected_components(graph)

    for comp_nodes in comps:
        path_pts = extract_best_path_from_component(graph, comp_nodes)
        if len(path_pts) < 2:
            continue

        endpoints, branch_nodes = component_stats(graph, comp_nodes)
        layer = dominant_layer_for_component(graph, comp_nodes, edge_meta)
        avg_style = style_score_for_component(graph, comp_nodes, edge_meta)
        ann = annotation_features_for_polyline(path_pts, annotations)

        candidatas.append({
            "tipo": "LINE_PATH",
            "capa": layer or "AUTO_LINE_GRAPH",
            "puntos": path_pts,
            "longitud": longitud(path_pts),
            "diag": bbox_diagonal(path_pts),
            "cerrada": False,
            "branch_nodes": branch_nodes,
            "endpoints": endpoints,
            "style_score": avg_style,
            "near_count": ann["near_count"],
            "near_weight_sum": ann["near_weight_sum"],
            "coverage": ann["coverage"],
            "weighted_coverage": ann["weighted_coverage"],
        })

    return candidatas

def puntuar_candidata(c):
    score = 0.0
    capa = (c["capa"] or "").lower()

    score += layer_score(capa)
    score += c.get("style_score", 0.0)

    # Señal principal: información pegada a la ruta
    score += min(c.get("near_weight_sum", 0.0) * 16.0, 1200.0)
    score += c.get("coverage", 0.0) * 900.0
    score += min(c.get("weighted_coverage", 0.0) * 220.0, 800.0)

    # Forma de ruta
    if c["cerrada"]:
        score -= 220.0
    else:
        score += 120.0

    n = len(c["puntos"])
    if n >= 4:
        score += min(n * 3.0, 120.0)
    else:
        score -= 60.0

    score += min(c["longitud"] * 0.08, 180.0)

    # Penalizaciones de base map / red
    if c["diag"] > 5000:
        score -= 180.0

    branch_nodes = c.get("branch_nodes", 0)
    score -= min(branch_nodes * 45.0, 550.0)

    endpoints = c.get("endpoints", 0)
    if endpoints == 2:
        score += 180.0
    elif endpoints == 0:
        score -= 120.0
    else:
        score -= min(abs(endpoints - 2) * 20.0, 120.0)

    if any(w in capa for w in GOOD_LAYER_WORDS):
        score += 260.0

    if any(w in capa for w in BAD_LAYER_WORDS):
        score -= 280.0

    if c["tipo"] in ("LWPOLYLINE", "POLYLINE", "SPLINE"):
        score += 60.0

    return score

def buscar_ruta(doc):
    annotations = extract_annotations(doc)
    candidatas = recolectar_candidatas(doc, annotations)
    if not candidatas:
        return None

    for c in candidatas:
        c["score"] = puntuar_candidata(c)

    candidatas.sort(
        key=lambda x: (
            x["score"],
            x.get("near_weight_sum", 0.0),
            x.get("coverage", 0.0),
            x["longitud"],
        ),
        reverse=True
    )

    log("Top 10 candidatas detectadas:")
    for i, c in enumerate(candidatas[:10], 1):
        log(
            f"{i}. tipo={c['tipo']}, capa={c['capa']}, puntos={len(c['puntos'])}, "
            f"longitud={c['longitud']:.2f}, cerrada={c['cerrada']}, ramas={c.get('branch_nodes', 0)}, "
            f"extremos={c.get('endpoints', 0)}, datos={c.get('near_count', 0)}, "
            f"peso_datos={c.get('near_weight_sum', 0.0):.2f}, cobertura={c.get('coverage', 0.0):.2f}, "
            f"style={c.get('style_score', 0.0):.2f}, score={c['score']:.2f}"
        )

    mejor = candidatas[0]
    log(
        f"Ruta elegida: tipo={mejor['tipo']}, capa={mejor['capa']}, "
        f"puntos={len(mejor['puntos'])}, longitud={mejor['longitud']:.2f}, "
        f"datos={mejor.get('near_count', 0)}, peso_datos={mejor.get('near_weight_sum', 0.0):.2f}, "
        f"cobertura={mejor.get('coverage', 0.0):.2f}, score={mejor['score']:.2f}"
    )
    return mejor["puntos"]


# =========================
# TRANSFORMACIÓN A WGS84
# =========================

def cargar_transformer():
    try:
        from pyproj import Transformer
        return Transformer
    except Exception as e:
        raise RuntimeError(
            "Falta la librería 'pyproj'. Instálala una sola vez con: pip install pyproj\n"
            f"Detalle: {e}"
        )

def dentro_de_region(lat, lon):
    return (
        REGION_BOUNDS["lat_min"] <= lat <= REGION_BOUNDS["lat_max"]
        and REGION_BOUNDS["lon_min"] <= lon <= REGION_BOUNDS["lon_max"]
    )

def centroide(puntos_geo):
    lat = sum(p[0] for p in puntos_geo) / len(puntos_geo)
    lon = sum(p[1] for p in puntos_geo) / len(puntos_geo)
    return lat, lon

def transformation_is_plausible(puntos_geo):
    if not puntos_geo:
        return False

    inside = sum(1 for lat, lon in puntos_geo if dentro_de_region(lat, lon))
    ratio = inside / len(puntos_geo)

    lat_c, lon_c = centroide(puntos_geo)
    if not dentro_de_region(lat_c, lon_c):
        return False

    return ratio >= 0.8

def score_transformacion(puntos_geo):
    if not puntos_geo:
        return -10**9

    inside = sum(1 for lat, lon in puntos_geo if dentro_de_region(lat, lon))
    lat_c, lon_c = centroide(puntos_geo)

    # Centro aproximado solo para priorizar, no como regla dura
    lat_ref = -25.3
    lon_ref = -57.6
    dist = math.hypot(lat_c - lat_ref, lon_c - lon_ref)

    return inside * 1000 - dist

def transformar_automaticamente_a_wgs84(doc, puntos_xy):
    # 1) Intentar GEODATA
    puntos_geo, origen = try_geodata_transform(doc, puntos_xy)
    if puntos_geo is not None:
        lat_c, lon_c = centroide(puntos_geo)
        log(f"Centro de la ruta: lat={lat_c:.6f}, lon={lon_c:.6f}")
        return puntos_geo, origen

    # 2) Probar CRS candidatos
    Transformer = cargar_transformer()
    mejores = []

    for epsg in CANDIDATE_EPSG:
        try:
            transformer = Transformer.from_crs(f"EPSG:{epsg}", "EPSG:4326", always_xy=True)
            puntos_geo = []
            for x, y in puntos_xy:
                lon, lat = transformer.transform(x, y)
                puntos_geo.append((lat, lon))

            sc = score_transformacion(puntos_geo)
            lat_c, lon_c = centroide(puntos_geo)
            log(f"Probando EPSG:{epsg} -> centro aprox=({lat_c:.6f}, {lon_c:.6f}) score={sc:.2f}")
            mejores.append((sc, epsg, puntos_geo))
        except Exception as e:
            log(f"No se pudo probar EPSG:{epsg}: {e}")

    if not mejores:
        raise RuntimeError("No se pudo transformar automáticamente las coordenadas a WGS84.")

    mejores.sort(key=lambda x: x[0], reverse=True)
    _, mejor_epsg, mejor_geo = mejores[0]

    lat_c, lon_c = centroide(mejor_geo)
    log(f"CRS elegido automáticamente: EPSG:{mejor_epsg}")
    log(f"Centro de la ruta: lat={lat_c:.6f}, lon={lon_c:.6f}")

    if not transformation_is_plausible(mejor_geo):
        raise RuntimeError(
            f"La transformación automática no resultó plausible "
            f"(lat={lat_c:.6f}, lon={lon_c:.6f})."
        )

    return mejor_geo, mejor_epsg


# =========================
# EXPORTACIÓN
# =========================

def link_google_maps_zona(puntos_geo):
    lat_c, lon_c = centroide(puntos_geo)
    return f"https://www.google.com/maps/@{lat_c:.6f},{lon_c:.6f},{GOOGLE_MAPS_ZOOM}z"

def generar_kml(nombre, puntos_geo):
    coords = "\n".join(f"{lon},{lat},0" for lat, lon in puntos_geo)
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>{nombre}</name>
    <Style id="rutaStyle">
      <LineStyle>
        <color>ff0000ff</color>
        <width>4</width>
      </LineStyle>
    </Style>
    <Placemark>
      <name>{nombre}</name>
      <styleUrl>#rutaStyle</styleUrl>
      <LineString>
        <tessellate>1</tessellate>
        <coordinates>
{coords}
        </coordinates>
      </LineString>
    </Placemark>
  </Document>
</kml>
'''

def guardar_kmz(path_kmz, kml):
    with zipfile.ZipFile(path_kmz, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("doc.kml", kml)


# =========================
# MAIN
# =========================

def main():
    Path("debug_ruta.log").write_text("", encoding="utf-8")
    try:
        base = Path(__file__).resolve().parent
    except NameError:
        base = Path.cwd().resolve()

    log(f"Carpeta: {base}")

    archivos = list(base.glob("*.dwg")) + list(base.glob("*.dxf"))
    if not archivos:
        raise RuntimeError("No se encontró ningún DWG o DXF en la carpeta.")

    archivo = archivos[0]
    log(f"Archivo detectado: {archivo.name}")

    ezdxf = cargar_ezdxf()
    doc = abrir_cad(archivo, ezdxf)
    log("Archivo abierto correctamente.")

    ruta = buscar_ruta(doc)
    if not ruta:
        raise RuntimeError("No se encontró ninguna entidad que pueda tomarse como ruta.")

    puntos_geo, origen_crs = transformar_automaticamente_a_wgs84(doc, ruta)

    nombre = archivo.stem
    url = link_google_maps_zona(puntos_geo)
    kml = generar_kml(nombre, puntos_geo)

    txt_path = base / f"{nombre}_google_maps_link.txt"
    kmz_path = base / f"{nombre}_ruta.kmz"

    txt_path.write_text(url + "\n", encoding="utf-8")
    guardar_kmz(kmz_path, kml)

    log(f"Origen CRS usado: {origen_crs}")
    log(f"KMZ generado: {kmz_path}")
    log(f"Link generado: {url}")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        log(f"ERROR: {e}")
        log(traceback.format_exc())