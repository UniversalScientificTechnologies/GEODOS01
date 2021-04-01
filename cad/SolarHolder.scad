$fn=100;
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
        cube([55, 100, 100]);
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
    width = 55;
    length = 70;
    angle = 35;
    
	difference(){
        cube([width, length, length*2]);
        rotate([angle,0,0]) translate([-1,0,0]) cube([width*2, length*2, length*2]);
        
        rotate([-90,0,0]) translate([width/2,0,0]) support_screw_end(3, -17.1, length);
        //#rotate([-90,0,0]) translate([width/2,0,0]) support_screw_end(1, -30.1, length);
        translate([width/2, length/2, -10]) rotate([0,0,0]) cylinder(100, 8.6/2, 8.6/2);

    }
}

SolarHolder();

