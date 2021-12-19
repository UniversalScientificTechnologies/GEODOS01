$fn=30;
INCH = 25.4;
MLAB_G = 10.16;

module support_screw_end(Nuts, Wall, Height){
	for(nut = [0 : Nuts-1]){
	translate([-(Nuts-1)/2*MLAB_G + nut*MLAB_G, -1.5/2*INCH-Wall+6, Height/2]) 
			rotate([90,0,0]) cylinder(10, 3.7/2, 3.7/2);
	translate([-(Nuts-1)/2*MLAB_G + nut*MLAB_G, -1.5/2*INCH-Wall+10+2, Height/2])
			rotate([90,0,0]) cylinder(10, 7.8/2, 7.8/2);
	translate([-(Nuts-1)/2*MLAB_G + nut*MLAB_G, -1.5/2*INCH-Wall-1, Height/2])
			rotate([90,0,0]) cylinder(100, 7.8/2, 7.8/2,$fn=6);
	}
}

   
module MlabSolarHolder(){
	Height = 10;
	Wall = 100;
	Nuts = 3;
	Distance = 5;
	Diameter = 1.5;
	
	difference(){
        //cube([55, 100, 100]);
//		difference(){
//			translate([-(Distance*MLAB_G)/2-MLAB_G/2,-50,0]) cube([(Distance*MLAB_G)+MLAB_G,50,100]);
//			rotate(a=[-35,0,0]) translate([-(Distance*MLAB_G)/2-MLAB_G/2,-100,-100]) #cube([(Distance*MLAB_G)+MLAB_G,50,300]);
	//	}
		translate([-(Diameter/2*INCH+Wall),0,0]) cube([(Diameter*INCH/2+Wall)*2,Diameter*INCH/2+Wall,10]);
		support_screw_end(Nuts, Wall, Height);
		
		for(dist = [0 : Distance]){
			translate([-(5)/2*MLAB_G + dist*MLAB_G,0,Height/2]) 
					rotate([90,0,0]) cylinder(Wall, 3.8/2, 3.5/2);
			translate([-(5)/2*MLAB_G + dist*MLAB_G,-2,Height/2])
					rotate([90,0,0]) #cylinder(Wall, 1.6/2, 1.6/2, $fn=6);
		}
	}
}


module SolarHolder(){
    width = 51;
    length = 81;
    angle = 35;
    border = 3;
    wall = 4.2;
    
	difference(){
        //cube([width, length, length*2]);
        union()
        {
            rotate([angle,0,0]) translate([0,0,-wall]) cube([width+border, length+border, 2*wall]); // horni
            rotate([0,0,0]) translate([0,0,0]) cube([width+border, (length+border)*cos(angle), border/2]); // dolni
            rotate([90,0,0]) translate([0,0,-(length+border)*cos(angle)]) cube([width+border, (length+border)*sin(angle), wall]); // zadni           
        }
        rotate([0,0,0]) translate([-border/2,0,-2*border]) cube([width+2*border, (length+border)*cos(angle), 2*border]); // spodni rovina
        rotate([angle,0,0]) translate([border/2,border/2,1.6]) cube([width, length, wall*2]); // ramecek pod clankem
        rotate([angle,0,0]) translate([border*2,border*2,-5*wall]) cube([width-3*border, length-3*border, wall*6]); // dira pod clankem 
        rotate([90,0,0]) translate([3,3,-(length+border)*cos(angle)-wall]) cube([width+border-6, (length+border)*sin(angle)-7, 2*wall+0.1]); // dira zadni  
        
        //rotate([angle-90,0,0]) translate([(width+border)/2,30,0]) support_screw_end(3, 0, length); // holes
        //#rotate([-90,0,0]) translate([width/2,0,0]) support_screw_end(1, -30.1, length);
        //translate([width/2, length/2, -10]) rotate([0,0,0]) cylinder(100, 8.6/2, 8.6/2);
 MLAB_grid_xoffset = 11.5;//3.88;
MLAB_grid_yoffset = 10;
MLAB_grid = 10.16;       
pedestal_height = 2;   // designed for use the MLAB standard 12mm screws.
mount_hole = 3.5;
clear = 0.175;        

            // MLAB grid holes
    grid_list = [for (j = [MLAB_grid_xoffset : MLAB_grid: width], i = [MLAB_grid_yoffset :MLAB_grid: (length+border)*cos(angle)]) [j, i] ];
    for (j = grid_list) {
            translate (concat(j, [0]))
            cylinder (h = 2*pedestal_height, r= mount_hole/2);
    }

    }
}

SolarHolder();

