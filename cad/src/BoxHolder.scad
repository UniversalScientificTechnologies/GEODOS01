roztec_X=35.6;
roztec_Y=38.1;
vyska=35;
vyska_obruby=10;
delka_sroubu=35;
kolik_sroubu_kouka=6;

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
    //valec se zavitem
    union()  
        {
        difference()
        {
            union()
            {
            
            //spodní doraz
            cylinder (h = S01_sila_materialu, r=S01_prumer_vnitrni/2+5/2*S01_sila_materialu, $fn=100);  


            //krycí ovál - usnadnění povolení
            difference()
            {
            cylinder (h = vyska_obruby, r=S01_prumer_vnitrni/2+5/2*S01_sila_materialu, $fn=100); 

            cylinder (h = vyska_obruby+0.01, r=S01_prumer_vnitrni/2+3/2*S01_sila_materialu, $fn=100);            
            
            }    
            }            

        //odstranění vnitřní výplně
        translate([0,0,S01_sila_materialu])
            cylinder (h = vyska_obruby+0.01, r=S01_prumer_vnitrni/2-S01_hloubka_zavitu/2-S01_sila_materialu, $fn=100); 
            
        //otvor na ložisko         
        translate([0,0,S01_sila_materialu/2])           
            cylinder (h = S01_sila_materialu+0.01, r=(lozisko_prumer_vnejsi+0.2)/2, center = true, $fn=100);          
        //otvory na hlavu šroubu ve dně 
        //otvor na hlavu šroubu 1
        translate([-roztec_X/2,-roztec_Y/2,(vyska_obruby)/2])           
            cylinder (h = vyska_obruby+0.01, r=(prumer_hlavy_sroubu)/2, center = true, $fn=100);
            
        //otvor na šroub 2 
        translate([roztec_X/2,-roztec_Y/2,(vyska_obruby)/2])           
            cylinder (h = vyska_obruby+0.01, r=(prumer_hlavy_sroubu)/2, center = true, $fn=100);           
            
        //otvor na šroub 3
        translate([roztec_X/2,roztec_Y/2,(vyska_obruby)/2])           
            cylinder (h = vyska_obruby+0.01, r=(prumer_hlavy_sroubu)/2, center = true, $fn=100);

        //otvor na šroub 4
        translate([-roztec_X/2,roztec_Y/2,(vyska_obruby)/2])           
            cylinder (h = vyska_obruby+0.01, r=(prumer_hlavy_sroubu)/2, center = true, $fn=100);          
     
        }

    //otvor na ložisko
    difference()
    {  
    translate([0,0,(vyska_obruby)/2])   
        cylinder (h = vyska_obruby, r=(lozisko_prumer_vnejsi+2*S01_sila_materialu)/2, center = true, $fn=100);     
    
    translate([0,0,vyska_obruby/2])
      cylinder (h = vyska_obruby+0.01, r=(lozisko_prumer_vnejsi+0.2)/2, center = true, $fn=100);
      
   

    }

    difference()
    { 
        union()
        { 
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
        
        translate([0,0,0])    
            cylinder (h = R01_vyska_preryti_statoru+lozisko_vyska+S01_sila_materialu, r=(lozisko_prumer_vnejsi+0.2)/2, $fn=100);  
   
       
    
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


  
WINDGAUGE02A_S01(); 

