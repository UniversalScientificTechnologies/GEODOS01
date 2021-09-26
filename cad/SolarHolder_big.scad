// source - https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/Primitive_Solids#polyhedron
module prism(l, w, h){
	polyhedron(
    	points=[[0,0,0], [l,0,0], [l,w,0], [0,w,0], [0,w,h], [l,w,h]],
        faces=[[0,1,2,3],[5,4,3,2],[0,4,5,1],[0,3,4],[5,2,1]]
   );
}

// source - https://github.com/ThunderFly-aerospace/TF-G2/blob/5dd7c33e797f1e813cab132e38739af15a9d39a1/CAD/bolt_parameters.scad
screw_diameter = 3.2;
nut_diameter = 6.6;

$fn = 6;
angle = 30;
hldr_thickness = 5;

module HolderPanel() {
	thickness = 4;
	panel_thickness = 6;
	width = 20;
	hldr_length = 20;
	panel_margin = 10;
	length = 60;
	
	triangle_size = 7;

	difference() {
		union() {
			translate([0, 0, thickness + panel_thickness]) {
				cube([length / cos(angle), width, thickness]);
			}
			
			cube([length / cos(angle), width, thickness]);
			
			cube([hldr_length, width, 2 * thickness + panel_thickness]);
		}
		
		translate([hldr_length + (length - hldr_length) / 2, panel_margin + (width - panel_margin) / 2, -1]) {
			#cylinder(r = screw_diameter / 2, h = panel_thickness + thickness * 2 + 2);
			translate([0, 0, panel_thickness + thickness * 2]){
				cylinder(r = nut_diameter / 2, h = 2);
			}
		}
	}

	translate([length/2, hldr_thickness + triangle_size, 0]) rotate ([180, 0, 0]) {
		prism(triangle_size, triangle_size, triangle_size);
	}
	translate([length/4, hldr_thickness + triangle_size, 0]) rotate ([180, 0, 0]) {
		prism(triangle_size, triangle_size, triangle_size);
	}
	translate([3*length/4, hldr_thickness + triangle_size, 0]) rotate ([180, 0, 0]) {
		prism(triangle_size, triangle_size, triangle_size);
	}
	translate([length, hldr_thickness + triangle_size, 0]) rotate ([180, 0, 0]) {
		prism(triangle_size, triangle_size, triangle_size);
	}

}

module SolarHolder() {
	height = 30;
	length = 60;
	prism_height = sqrt(3) / 3 * length;

	difference() {
		cube([length, hldr_thickness, height]);
		translate([length / 3, hldr_thickness + 1, height / 2]) rotate([90, 0, 0]) {
			#cylinder(r = screw_diameter / 2, h = hldr_thickness + 2);
			cylinder(r = nut_diameter / 2, h = 2);
		}
		translate([2*length / 3, hldr_thickness + 1, height / 2]) rotate([90, 0, 0]) {
			#cylinder(r = screw_diameter / 2, h = hldr_thickness + 2);
			cylinder(r = nut_diameter / 2, h = 2);
		}
	}

	translate([length, 0, height]) rotate([0, 0, 90]) {
		prism(hldr_thickness, length, prism_height);	
	}
	
	translate([0, 0, height + prism_height]) rotate([0, angle, 0]){
		HolderPanel();
	}
}

SolarHolder();

translate([-10, 0, 0]) mirror([1,0,0]) {
    SolarHolder();
}

//HolderPanel();
