#!/bin/bash

name="Robot"
timestamp=$(gdate +"%Y-%m-%dT%H:%M:%S.%3NZ")
source=robot.py

tmpdir=$(mktemp -d -t package)

cat manifest.template \
  | perl -pe "s/{{name}}/$name/g" \
  | perl -pe "s/{{timestamp}}/$timestamp/g" \
  > $tmpdir/manifest.json

echo -n '{ "program": ' > $tmpdir/projectbody.json
cat $source \
  | python -c 'import json,sys; print(json.dumps(sys.stdin.read()))' \
  >> $tmpdir/projectbody.json
echo ' }' >> $tmpdir/projectbody.json

zip -j -9 ${tmpdir}/${name}.llsp $tmpdir/manifest.json $tmpdir/projectbody.json icon.svg

cp ${tmpdir}/${name}.llsp ~/Documents/LEGO\ Education\ SPIKE/

rm -rf $tmpdir
