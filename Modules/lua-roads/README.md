# lua-roads
Parses an OSM file and extracts road information

## Installation

```sh
luarocks make
```

Install [lua-cjson](https://github.com/openresty/lua-cjson) for saving JSON output.

Install the [jq](https://stedolan.github.io/jq/) utility for inspecting output.

# Usage

Acquire an OSM map:

```sh
BASENAME=test
MIN_LON=-71.6094
MAX_LON=-71.6010
MIN_LAT=42.5187
MAX_LAT=42.5247
OVERPASS_BBOX="$MIN_LAT,$MIN_LON,$MAX_LAT,$MAX_LON"
OVERPASS_QUERY="[bbox:$OVERPASS_BBOX];(way[lanes][highway!=footway];way[highway][highway!=footway];);(._;>;);out;"
wget "http://overpass-api.de/api/interpreter?data=$OVERPASS_QUERY" -O $BASENAME.osm
```

Form roads:

```sh
roads.lua $BASENAME.osm
```
