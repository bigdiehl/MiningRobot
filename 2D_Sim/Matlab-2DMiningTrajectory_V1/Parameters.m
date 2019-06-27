
mToInches = 39.3701;

%Scaling factor = Pixels/in
scale = 1;

%Reference Geometry
P.turning_radius = 4.0*mToInches*scale; %in

%Map scale
P.map_x_max = 200*scale;
P.map_y_max = 200*scale;

%Cutter head dimensions (works better if cutter width is a little larger than front width)
P.cutter_length = 3.75*scale;
P.cutter_width = 6.0*scale;

%Interbody linkage dimensions and parameters
P.linkage_max_extension = 4.0*scale;
P.linkage_width = 2.0*scale;
P.linkage_length = (P.linkage_max_extension+2)*scale;
P.linkage_x_offset = P.linkage_length/2 - 1.0;

%Back body dimensions
P.back_length = 42.0*scale;
P.back_width = 9.0*scale;

%Front body dimensions
P.front_length = 32.0*scale;
P.front_width = 4.0*scale;

%Initial tunnel dimensions (Centered about robot)
P.init_tunnel_length = (P.front_length+P.cutter_length)*scale;
P.init_tunnel_width = 6.0*scale;

%Bit dimension
P.bit_length = 4.0*scale;

%Movement 
P.d1_max = 4.0*scale;
P.d1_min = 0.0*scale;

%Bracing
P.back_bracing_width = 1.0*scale;
P.front_bracing_width = 1.0*scale;

P.front_bracing_ext_max = 1.5*scale;
P.back_bracing_ext_max = 1.5*scale;

%Initial Box location (pixels)
P.x0 = 20*scale;
P.y0 = 180*scale;
