$fn=50; // Set the number of facets for the ellipsoid

// Initialize ellipsoid width, height, and length from command-line arguments

rotate([0, -90, 0]) // Rotate around the z-axis by 0 degrees
translate([length/2,0,0]) // Rotate around the y-axis
scale([length/2, width/2, height/2]) sphere(r=1); // Generate the ellipsoid
