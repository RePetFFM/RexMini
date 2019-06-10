$fn = 12;
teeth = 35; // 64,68mm
motor_shaft = 20;
pulley_t_ht = 8;

include <Pulley_T-MXL-XL-HTD-GT2_N-tooth.scad>

union() {
    translate([0,0,-6]) rotate([0,-90,0]) {
        import("rim_d.stl");
    }

	translate([0,0,5]) pulley (  "MXL" , MXL_pulley_dia , 0.508 , 1.321 );
    difference() {
        translate([0,0,4.5]) cylinder(r=15,h=0.5,$fn=64);
        cylinder(r=10,h=30,$fn=64);	
    }
}