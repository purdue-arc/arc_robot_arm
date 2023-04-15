#!/bin/bash

# Set the ellipsoid parameters
width=2
height=2
lengths=(4.5 11.2 11.0 6.8 12.8)

for ((i=1; i<=${#lengths[@]}; i++)); do
  length=${lengths[i-1]}
  output_file="link_$i.stl"
  
  # Call OpenSCAD with the ellipsoid parameters
  openscad -o "$output_file" -D "width=$width" -D "height=$height" -D "length=$length" ellipsoid.scad
done

