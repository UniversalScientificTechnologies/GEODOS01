roztec_X=35.6;
roztec_Y=38.1;
vyska=35;
vyska_obruby=10;
delka_sroubu=35;
kolik_sroubu_kouka=4.8;

//Parametry sroubu
prumer_sroubu=3.4;
prumer_hlavy_sroubu=6.4;

vyska_matky=3;
sirka_matky=6.7;



$fn=40; // model faces resolution.

//OBECNE PARAMETRY 
//----------------------------------------------------------------
//----------------------------------------------------------------
//WINDGAUGE02A_S02 - Parametry pro hlavni valec anemometru
S01_hloubka_zavitu=4;

S01_prumer_vnitrni=37.5;
S01_sila_materialu=2;
S01_vyska_horni_zavit=5;

//Obecné parametry

//Parametry loziska

lozisko_prumer_vnejsi=16.1;
lozisko_vyska=5;


//Parametry magnetu

magnet_vyska=5;

//WINDGAUGE02A_R01 //rotor s prekritim statoru
R01_vyska_preryti_statoru=20;


//Držák ložisek, rotoru, senzoru

module WINDGAUGE02A_S01()
{
    difference()
    {
        union()
        { 
        translate([0,0,S01_sila_materialu/4]) 
            minkowski()
            {
                cube([50,50,S01_sila_materialu/2], center=true);
                cylinder(r=4, h=0.0001);
            }
        //uchyty na modul
        //sloupek 1
        translate([-roztec_X/2,-roztec_Y/2,0]) 
            SLOUPEK();

        translate([roztec_X/2,-roztec_Y/2,0])  
            SLOUPEK();
        
        translate([roztec_X/2,roztec_Y/2,0])  
            SLOUPEK();

        translate([-roztec_X/2,roztec_Y/2,0])  
            SLOUPEK();
        }    
        union()
        { 
        translate([-roztec_X/2,-roztec_Y/2,0]) 
            dira();

        translate([roztec_X/2,-roztec_Y/2,0])  
            dira();
        
        translate([roztec_X/2,roztec_Y/2,0])  
            dira();

        translate([-roztec_X/2,roztec_Y/2,0])  
            dira();
        }    
    }
}



//sloupek na senzor
module SLOUPEK()
{    
translate([0,0,0]) 
    difference () 
    {       
        cylinder (h = vyska, r= (sirka_matky+1)/2+S01_sila_materialu/2, $fn=20);    
    }       
}

module dira()
{    
translate([0,0,0]) 
    union () 
    {        
        translate([0,0,vyska-vyska_matky]) 
            cylinder (h = vyska_matky+0.01, r= (sirka_matky+0.2)/2, $fn=6);
        
        translate([0,0,-0.01]) 
            cylinder (h = vyska+0.01, r= (prumer_sroubu+0.2)/2, $fn=40);
               
        translate([0,0,-0.01]) 
            cylinder (h = vyska-(delka_sroubu-kolik_sroubu_kouka)+0.02, r= (prumer_hlavy_sroubu)/2, $fn=40);
    
        translate([0,0,vyska-(delka_sroubu-kolik_sroubu_kouka)]) 
            cylinder(h=2+0.02, r1=(prumer_hlavy_sroubu)/2, r2=(prumer_sroubu+0.2)/2);     
    }       
}

  
difference()
{
    WINDGAUGE02A_S01(); 

}
