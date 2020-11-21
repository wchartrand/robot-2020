#!/bin/bash

name="Robot"
timestamp=$(gdate +"%Y-%m-%dT%H:%M:%S.%3NZ")
source=robot.py

unzip -p ~/Documents/LEGO\ Education\ SPIKE/${name}.llsp projectbody.json \
  | python -c 'import json,sys; print(json.loads(sys.stdin.read())["program"])' \
  > ${source}
  
