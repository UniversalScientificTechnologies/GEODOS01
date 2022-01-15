//use<Write.scad>
$fn=100;

x_size = 20;           // horizontal outer size of the pedestal. 
y_size = 20;           // 
nut_size = 6.8;
SMA_dia = 7.0;
SMA_flat = 3.15;
thickness = 6;         //
wall_thickness = 2;
height = 55;

mount_hole = 3.7;
clear = 0.175;

MLAB_grid = 10.16;

x_holes = floor(x_size / MLAB_grid);
y_holes = floor(y_size / MLAB_grid);

MLAB_grid_xoffset = (x_size - (x_holes * MLAB_grid))/2;
MLAB_grid_yoffset = (y_size - (y_holes * MLAB_grid))/2;

//Top part

union () {

    difference () {
                cube([x_size, y_size , thickness ]);

// MLAB grid holes
        grid_list = [for (j = [MLAB_grid_xoffset : MLAB_grid: x_size], i = [MLAB_grid_yoffset :MLAB_grid: y_size])  [j, i] ];
            
        for (j = grid_list) {
                translate (concat(j, [0]))
                    cylinder (h = 3, r= nut_size/2, $fn=6);
                translate (concat(j, [3.2]))  // one solid layer for slicer (the holes will be pierced on demand )
                    cylinder (h = thickness, r= mount_hole/2, $fn=30);
                //translate (concat(j, [6.0]))
                //    cylinder (h = 10, r= nut_size/2, $fn=6);
        }  
    };
// vertical to horizontal brick connection
    translate ([ 0, 0, thickness - sqrt(2 * wall_thickness*wall_thickness) / 2])  // move to correct positon on square diagonal  
        rotate([45,0,0])
            cube([x_size, wall_thickness , wall_thickness ]);
// rib
    difference()
    {
        translate ([ (x_size / 2) - wall_thickness / 2, 0, -y_size ])  // move to correct positon on square diagonal  
            rotate([45,0,0])
                cube([wall_thickness, y_size*1.4, y_size*1.4]);
        translate ([ 0, -y_size , -thickness * 4]) cube([2*x_size, 2*y_size , thickness * 4 ]);
        translate ([ 0, -y_size - wall_thickness, 0]) cube([x_size, y_size , thickness * 3 ]);
    };
    
    difference  () {

        translate ([0, -wall_thickness, 0])    // vertical brick
            cube([y_size, wall_thickness , height ]);
        
// SMA connector hole
        translate ([ x_size/2, wall_thickness/2, 1.5*(height - thickness)/3  + thickness - SMA_dia/2])    
            rotate([90,0,0]) {
                difference  () {
                    cylinder (h = 2*wall_thickness, r= SMA_dia/2, $fn=50);
                        translate([ 0, SMA_flat , wall_thickness/2 + wall_thickness ]) 
                            cube([SMA_dia, 1 , thickness ], center=true);
                }
            }

        translate ([ x_size/2, wall_thickness/2, 2.3*(height - thickness)/3  + thickness  + SMA_dia/2 ])    
            rotate([90,0,0]){
                difference  () {
                    cylinder (h = 2*wall_thickness, r= SMA_dia/2, $fn=50);
                        translate([ 0, SMA_flat , wall_thickness/2 + wall_thickness ]) 
                            cube([SMA_dia, 1 , thickness ], center=true);
                }
 
           }
           
    }
               
           
}

