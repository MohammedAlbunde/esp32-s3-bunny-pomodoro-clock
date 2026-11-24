/*
  Bunny Pomodoro enclosure for Waveshare ESP32-S3-Touch-LCD-1.28

  Designed from Waveshare's published outline dimensions:
    touch lens OD: 38.51 mm
    viewable area: 33.40 mm
    overall height including USB feature: about 40.36 mm

  This is a fit-test-first prototype. Board revisions and printer calibration
  vary, so print bunny-board-fit-test.stl before the full enclosure.

  Export with:
    openscad -o bunny-front.stl -D 'part="front"' bunny_pomodoro_case.scad
    openscad -o bunny-rear.stl  -D 'part="rear"'  bunny_pomodoro_case.scad
    openscad -o bunny-board-fit-test.stl -D 'part="fit_test"' bunny_pomodoro_case.scad
*/

$fn = 80;
part = "front"; // front, rear, fit_test, preview

// Board and screen parameters. Change only these for fit tuning.
glass_od = 38.51;
glass_clearance = 0.34;        // diametral clearance
board_pocket_d = glass_od + glass_clearance;
screen_opening_d = 34.50;     // clears the 33.40 mm active area

// Enclosure geometry. The two halves use ellipsoidal Minkowski rounding so
// they resemble inflated cushions rather than flat extrusions.
pillow_plate = 0.80;
outer_round_xy = 4.30;
front_bulge_z = 5.20;
front_thickness = front_bulge_z + pillow_plate/2;
front_lip = 1.55;
// Adds 12.7 mm (0.5 in) of battery space compared with Version 4.
rear_bulge_z = 24.20;
rear_depth = rear_bulge_z + pillow_plate/2;
rear_wall = 2.00;
case_gap = 0.25;
usb_slot_w = 13.0;
usb_slot_h = 9.2;
table_y = -29.5;              // slices both shells into one continuous flat base

// Hidden tapered relief for the lower USB section of the round board.
// The visible front opening remains a clean circle.
lower_relief_top_y = -12.5;
lower_relief_bottom_y = -24.0;
lower_relief_top_w = 26.0;
lower_relief_bottom_w = 19.0;

// M2 self-tapping screw geometry (screws 22-25 mm long).
// V6: the screws were moved from (+-23, 0) to (+-20, -19). At (23, 0) the rear
// shell top already slopes away (about 23.5 mm high instead of 24.6, and it
// drops to 12.9 mm under the head edge) and the post sits only 0.2 mm from
// the outer wall, so the head had no flat seat and the socket broke out of the
// side. In the lower, wider body the top is flat and the walls are thick.
screw_x = 20.0;
screw_y = -19.0;
boss_r = 3.05;
boss_socket_r = boss_r + 0.25;
pilot_d = 1.65;               // self-tapping thread in PLA
clearance_d = 2.35;           // free passage through the rear
counterbore_d = 4.40;         // M2 pan head is about 4.0 mm
head_seat_depth = 1.80;       // head (1.6 mm) sits flush, solid seat below it
post_embed = 1.5;             // post root sunk into the front shell
post_h = 8.05;
post_out = post_h - post_embed;   // post length entering the rear (6.55)
socket_h = post_out + 0.6;        // rear socket only as deep as the post
pilot_bottom_z = 2.6;         // pilot hole depth in front coordinates:
                              // fits M2 screws from 22 up to about 25.8 mm

module rounded_rect_2d(w, h, r) {
    hull() {
        for (x = [-w/2+r, w/2-r])
            for (y = [-h/2+r, h/2-r])
                translate([x, y]) circle(r=r);
    }
}

module ear_2d(side) {
    hull() {
        translate([side*12.5, 19.5]) circle(r=7.4);
        translate([side*18.0, 34.8]) circle(r=7.1);
    }
}

module inner_ear_2d(side) {
    hull() {
        translate([side*13.2, 22.4]) circle(r=2.55);
        translate([side*17.3, 33.5]) circle(r=3.05);
    }
}

// The lower body is wider than the face and ends in a broad horizontal
// contact area. This creates the stable pear silhouette without feet.
module bunny_body_2d() {
    hull() {
        translate([0,1.5]) circle(r=26.0);
        translate([0,-25.5]) rounded_rect_2d(57.5,10.5,5.25);
    }
}

module bunny_outline_2d(inset=0) {
    offset(delta=-inset)
    union() {
        bunny_body_2d();
        ear_2d(-1);
        ear_2d(1);
    }
}

module usb_tunnel(z0, depth) {
    translate([-usb_slot_w/2, -34.0, z0])
        cube([usb_slot_w, 16.0, depth]);
}

module lower_device_relief_2d() {
    polygon(points=[
        [-lower_relief_top_w/2, lower_relief_top_y],
        [ lower_relief_top_w/2, lower_relief_top_y],
        [ lower_relief_bottom_w/2, lower_relief_bottom_y],
        [-lower_relief_bottom_w/2, lower_relief_bottom_y]
    ]);
}

// Full symmetrical pillow volume. Each printable half is sliced from its
// center plane so the seam remains flat and both outlines match.
module puffy_volume(inset=0, z_bulge=5.2) {
    minkowski() {
        linear_extrude(height=pillow_plate, center=true)
            bunny_outline_2d(inset=outer_round_xy+inset);
        scale([outer_round_xy, outer_round_xy, z_bulge])
            sphere(r=1);
    }
}

module front_outer() {
    translate([0,0,front_thickness])
        intersection() {
            puffy_volume(z_bulge=front_bulge_z);
            translate([-60,table_y,-front_thickness])
                cube([120,120,front_thickness]);
        }
}

module rear_outer() {
    intersection() {
        puffy_volume(inset=case_gap, z_bulge=rear_bulge_z);
        translate([-60,table_y,0]) cube([120,120,rear_depth]);
    }
}

module front_part() {
    difference() {
        union() {
            // Soft, rounded front shell.
            front_outer();

            // Two reinforced screw posts extending into the rear cover.
            for (x=[-screw_x, screw_x])
                translate([x, screw_y, front_thickness-post_embed])
                    cylinder(r=boss_r, h=post_h);
        }

        // Flared screen opening creates the rounded bezel seen in the mockup.
        translate([0,0,-0.1])
            cylinder(d1=37.10, d2=screen_opening_d, h=1.30);
        translate([0,0,1.19])
            cylinder(d=screen_opening_d, h=front_thickness+0.2);

        // Shallow board/glass locating pocket from the rear.
        translate([0,0,front_lip])
            cylinder(d=board_pocket_d, h=front_thickness-front_lip+0.2);

        // Wider tapered pocket for the board's lower USB/triangle section.
        translate([0,0,front_lip])
            linear_extrude(height=front_thickness-front_lip+0.2)
                lower_device_relief_2d();

        // USB-C cable exit. Leaves the cosmetic front lip intact.
        usb_tunnel(front_lip, front_thickness-front_lip+7.0);

        // Pilot holes for M2 self-tapping screws.
        for (x=[-screw_x, screw_x])
            translate([x,screw_y,pilot_bottom_z])
                cylinder(d=pilot_d, h=front_thickness+post_out-pilot_bottom_z+0.3);

        // Recessed inner-ear details, printable in one color.
        for (side=[-1,1])
            translate([0,0,-0.1])
                linear_extrude(height=0.90)
                    inner_ear_2d(side);

        // Subtle recessed cheeks, integrated into the one-color print.
        for (x=[-23.0,23.0])
            translate([x,0,-0.1])
                scale([1.65,1.10,1]) cylinder(r=1.0,h=0.75);
    }
}

module rear_part() {
    difference() {
        // Deep rounded rear shell, slightly inset for a clean center seam.
        rear_outer();

        // Main electronics cavity, open at the mating face.
        translate([0,0,-0.1])
            cylinder(d=41.20, h=rear_depth-rear_wall+0.1);

        // USB-C access from the bottom.
        usb_tunnel(-0.1, rear_depth+0.2);

        // Sockets for the front's reinforced screw posts.
        for (x=[-screw_x, screw_x])
            translate([x,screw_y,-0.1])
                cylinder(r=boss_socket_r, h=socket_h+0.1);

        // M2 clearance holes through the rear wall.
        for (x=[-screw_x, screw_x])
            translate([x,screw_y,-0.1])
                cylinder(d=clearance_d, h=rear_depth+0.2);

        // Recess screw heads into the back face.
        for (x=[-screw_x, screw_x])
            translate([x,screw_y,rear_depth-head_seat_depth])
                cylinder(d=counterbore_d, h=head_seat_depth+0.2);
    }
}

module fit_test_part() {
    // A fast, low-filament gauge for screen diameter and USB alignment.
    difference() {
        union() {
            cylinder(d=44.0, h=3.0);
            translate([0,-21.0,0]) rounded_rect_3d(18.0, 10.0, 3.0, 2.0);
        }
        translate([0,0,1.45])
            cylinder(d=board_pocket_d, h=1.70);
        translate([0,0,1.45])
            linear_extrude(height=1.70)
                lower_device_relief_2d();
        translate([0,0,-0.1])
            cylinder(d=screen_opening_d, h=3.2);
        translate([-usb_slot_w/2,-27.0,1.45])
            cube([usb_slot_w,12.0,1.7]);
    }
}

module rounded_rect_3d(w, h, d, r) {
    linear_extrude(height=d) rounded_rect_2d(w,h,r);
}

module assembled_preview() {
    color([0.95,0.93,0.88]) front_part();
    color([0.80,0.93,0.92,0.75])
        translate([0,0,front_thickness]) rear_part();
    // Visual-only LCD reference.
    color([0.03,0.04,0.06])
        translate([0,0,-0.25]) cylinder(d=38.51,h=0.22);
}

if (part == "front") front_part();
else if (part == "rear") rear_part();
else if (part == "fit_test") fit_test_part();
else assembled_preview();

