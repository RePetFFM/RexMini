$fn = 12;
teeth = 25; // 64,68mm
motor_shaft = 5;
pulley_t_ht = 9.5;

include <Pulley_T-MXL-XL-HTD-GT2_N-tooth.scad>

union() {
    translate([0,0,2.5]) rotate([0,0,0]) {
        import("motor_pulley_shaft2.stl");
    }

	translate([0,0,0]) pulley (  "MXL" , MXL_pulley_dia , 0.508 , 1.321 );
    difference() {
        translate([0,0,8.5]) hull() {
            translate([0,0,0.9]) cylinder(r=8.2,h=0.1,$fn=64);
            translate([0,0,0]) cylinder(r=7.25,h=0.1,$fn=64);
        }
        
        cylinder(r=5,h=30,$fn=64);	
    }
    
    hull() {
        cylinder(r=7.25,h=1.5,$fn=64);
        cylinder(r=9,h=0.5,$fn=64);
    }
}