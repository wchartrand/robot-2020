#!/bin/bash

name="Robot"
timestamp=$(gdate +"%Y-%m-%dT%H:%M:%S.%3NZ")
source=robot.py

cat manifest.template \
  | perl -pe "s/{{name}}/$name/g" \
  | perl -pe "s/{{timestamp}}/$timestamp/g" \
  > manifest.json

echo -n '{ "program": ' > projectbody.json
cat $source \
  | python -c 'import json,sys; print(json.dumps(sys.stdin.read()))' \
  >> projectbody.json
echo ' }' >> projectbody.json

zip -9 ${name}.llsp manifest.json projectbody.json icon.svg

cp ${name}.llsp ~/Documents/LEGO\ Education\ SPIKE/
