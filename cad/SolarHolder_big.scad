
// source - https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/Primitive_Solids#polyhedron
module prism(l, w, h){
	polyhedron(
    	points=[[0,0,0], [l,0,0], [l,w,0], [0,w,0], [0,w,h], [l,w,h]],
        faces=[[0,1,2,3],[5,4,3,2],[0,4,5,1],[0,3,4],[5,2,1]]
   );
}

screw_diameter = 3.2;
angle = 30;

module HolderPanel() {
	thickness = 5;
	panel_thickness = 6;
	width = 20;
	hldr_length = 20;
	panel_margin = 10;
	length = 60;

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
		}
	}
}

module SolarHolder() {
	thickness = 10;
	height = 100;
	length = 60;
	prism_height = sqrt(3) / 3 * length;

	difference() {
		cube([length, thickness, height]);
		translate([length / 2, thickness + 1, height / 3]) rotate([90, 0, 0]) {
			#cylinder(r = screw_diameter / 2, h = thickness + 2);
		}
		translate([length / 2, thickness + 1, 2 * height / 3]) rotate([90, 0, 0]) {
			#cylinder(r = screw_diameter / 2, h = thickness + 2);
		}
	}

	translate([length, 0, height]) rotate([0, 0, 90]) {
		prism(thickness, length, prism_height);	
	}
	
	translate([0, 0, height + prism_height]) rotate([0, angle, 0]){
		HolderPanel();
	}
}

SolarHolder();

translate([-10, 0, 0]) mirror([1,0,0]) {
    SolarHolder();
}

