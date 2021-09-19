
// source - https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/Primitive_Solids#polyhedron
module prism(l, w, h){
	polyhedron(
    	points=[[0,0,0], [l,0,0], [l,w,0], [0,w,0], [0,w,h], [l,w,h]],
        faces=[[0,1,2,3],[5,4,3,2],[0,4,5,1],[0,3,4],[5,2,1]]
   );
}

module SolarHolder() {
	thickness = 10;
	panel_margin = 10;
	panel_thickness = 6;
	holder_thickness = 5;
	hh_thickness = 20;
	holder_length = 10;
	screw_diameter = 3.2;
	height = 100;
	length = 60;
	
	angle = 30;
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

	translate([0, 0, height + sqrt(3) / 3 * length]) rotate([0, angle, 0]) {
		cube([holder_length,hh_thickness,panel_thickness]);
	}

	translate([length, 0, height]) rotate([0, 0, 90]) {
		prism(thickness, length, prism_height);	
	}
	
	difference() {
		union() {
			translate([panel_thickness * sin(angle), 0, height + prism_height + panel_thickness * cos(angle)]) rotate([0, angle, 0]) {
				cube([length / cos(angle), hh_thickness, holder_thickness]);
			}

			translate([0, 0, height + prism_height - holder_thickness]) {
				rotate([0, angle, 0]) {
					cube([length / cos(angle), hh_thickness, holder_thickness]);
				}
			
				cube([holder_length * sin(angle),hh_thickness,holder_thickness]);
			}

		}

		translate([length / 2, (hh_thickness - thickness) / 2 + thickness, height + prism_height * tan(angle) / 2 + 1]) {
			rotate([0, angle, 0]) {
				#cylinder(r = screw_diameter / 2, h = holder_thickness * 2 + panel_thickness + 2);
			}
		}	
	}
}

SolarHolder();

translate([-10, 0, 0]) mirror([1,0,0]) {
    SolarHolder();
}

