# De-autocad-a-maps
Converts AutoCAD route files (`.dxf` / `.dwg`) to a Google Maps link and a `.kmz` file.

## Usage

1. Place `analizador.py` in the **same folder** as your AutoCAD file.
2. Run it:

python analizador.py yourfile.dxf


3. Two files will be saved in the same folder:
   - `yourfile_google_maps_link.txt` — direct Google Maps URL
   - `yourfile_ruta.kmz` — route file for Google Earth or Maps

## Requirements

pip install ezdxf pyproj


> Designed for DXF/DWG files with UTM coordinates in the Paraguay region.
