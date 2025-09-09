% Simscape(TM) Multibody(TM) version: 7.6

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(13).translation = [0.0 0.0 0.0];
smiData.RigidTransform(13).angle = 0.0;
smiData.RigidTransform(13).axis = [0.0 0.0 0.0];
smiData.RigidTransform(13).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [0 0.10000000000000001 0];  % m
smiData.RigidTransform(1).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(1).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(1).ID = "B[Part1-1:-:Part2^Assem1-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [-0.070000000000013829 0.0075000000000000344 1.1199374760906267e-14];  % m
smiData.RigidTransform(2).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(2).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(2).ID = "F[Part1-1:-:Part2^Assem1-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0 -0.10000000000000001 0];  % m
smiData.RigidTransform(3).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(3).axis = [0.57735026918962584 0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(3).ID = "B[Part1-1:-:Part2^Assem1-3]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-0.069999999999967949 0.0074999999999999928 2.8838043064638441e-14];  % m
smiData.RigidTransform(4).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(4).axis = [0.57735026918962584 -0.57735026918962573 0.57735026918962573];
smiData.RigidTransform(4).ID = "F[Part1-1:-:Part2^Assem1-3]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [0.070000000000000007 0.014999999999999993 0];  % m
smiData.RigidTransform(5).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(5).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(5).ID = "B[Part2^Assem1-2:-:Part3-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [0.12499999999992648 7.7299278089526524e-15 2.7755575615628914e-17];  % m
smiData.RigidTransform(6).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(6).axis = [-1 7.1021041022648044e-33 -1.0338398897426087e-16];
smiData.RigidTransform(6).ID = "F[Part2^Assem1-2:-:Part3-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [-0.12499999999999997 0 -0.015000000000000003];  % m
smiData.RigidTransform(7).angle = 0;  % rad
smiData.RigidTransform(7).axis = [0 0 0];
smiData.RigidTransform(7).ID = "B[Part3-1:-:Part3-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [0.12500000000003267 3.1058489113888754e-14 2.7755575615628914e-17];  % m
smiData.RigidTransform(8).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(8).axis = [-1 -1.1793797763875355e-33 2.0083413447342084e-17];
smiData.RigidTransform(8).ID = "F[Part3-1:-:Part3-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [0.070000000000000007 0.014999999999999996 0];  % m
smiData.RigidTransform(9).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(9).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(9).ID = "B[Part2^Assem1-3:-:Part3-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [-0.12500000000009187 1.4932499681208355e-14 -0.015000000000000041];  % m
smiData.RigidTransform(10).angle = 1.7554167342883509e-16;  % rad
smiData.RigidTransform(10).axis = [0.97581324394308167 -0.2186058392296045 -1.8723140872695111e-17];
smiData.RigidTransform(10).ID = "F[Part2^Assem1-3:-:Part3-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(11).translation = [0.125 0 -0.014999999999999993];  % m
smiData.RigidTransform(11).angle = 0;  % rad
smiData.RigidTransform(11).axis = [0 0 0];
smiData.RigidTransform(11).ID = "B[Part3-2:-:Part5-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(12).translation = [-0.02499999999999936 -0.0099999999999963278 3.4694469519536142e-18];  % m
smiData.RigidTransform(12).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(12).axis = [1 1.2308645160719607e-33 5.8371277012136484e-17];
smiData.RigidTransform(12).ID = "F[Part3-2:-:Part5-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(13).translation = [0.027500509196102033 -0.0081048345994645353 -0.013630338883696368];  % m
smiData.RigidTransform(13).angle = 0;  % rad
smiData.RigidTransform(13).axis = [0 0 0];
smiData.RigidTransform(13).ID = "RootGround[Part1-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(4).mass = 0.0;
smiData.Solid(4).CoM = [0.0 0.0 0.0];
smiData.Solid(4).MoI = [0.0 0.0 0.0];
smiData.Solid(4).PoI = [0.0 0.0 0.0];
smiData.Solid(4).color = [0.0 0.0 0.0];
smiData.Solid(4).opacity = 0.0;
smiData.Solid(4).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 7.5026507188014664;  % kg
smiData.Solid(1).CoM = [0 0 49.981210182193337];  % mm
smiData.Solid(1).MoI = [62534.049637619399 45320.054874849076 95339.094164725364];  % kg*mm^2
smiData.Solid(1).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(1).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "Part1*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.10016249977391731;  % kg
smiData.Solid(2).CoM = [0 0 -7.5000000000000009];  % mm
smiData.Solid(2).MoI = [7.6324717586416213 562.53903072610001 566.4154087432197];  % kg*mm^2
smiData.Solid(2).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(2).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "Part3*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.057262499773917327;  % kg
smiData.Solid(3).CoM = [0 7.4999999999999991 0];  % mm
smiData.Solid(3).MoI = [4.4113967586416241 111.97749866795628 109.71344565083655];  % kg*mm^2
smiData.Solid(3).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(3).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "Part2^Assem1*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0.11515900570815087;  % kg
smiData.Solid(4).CoM = [-25.000000000002203 -9.999999999999142 -7.5];  % mm
smiData.Solid(4).MoI = [75.753033442393004 75.753033442393004 147.18760417073037];  % kg*mm^2
smiData.Solid(4).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(4).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "Part5*:*Default";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the CylindricalJoint structure array by filling in null values.
smiData.CylindricalJoint(1).Rz.Pos = 0.0;
smiData.CylindricalJoint(1).Pz.Pos = 0.0;
smiData.CylindricalJoint(1).ID = "";

%This joint has been chosen as a cut joint. Simscape Multibody treats cut joints as algebraic constraints to solve closed kinematic loops. The imported model does not use the state target data for this joint.
smiData.CylindricalJoint(1).Rz.Pos = 90.891096595693966;  % deg
smiData.CylindricalJoint(1).Pz.Pos = 0;  % m
smiData.CylindricalJoint(1).ID = "[Part3-1:-:Part3-2]";


%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(5).Rz.Pos = 0.0;
smiData.RevoluteJoint(5).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = -30.44137944938263;  % deg
smiData.RevoluteJoint(1).ID = "[Part1-1:-:Part2^Assem1-2]";

smiData.RevoluteJoint(2).Rz.Pos = -34.757933095629888;  % deg
smiData.RevoluteJoint(2).ID = "[Part1-1:-:Part2^Assem1-3]";

smiData.RevoluteJoint(3).Rz.Pos = 14.076207244031675;  % deg
smiData.RevoluteJoint(3).ID = "[Part2^Assem1-2:-:Part3-1]";

smiData.RevoluteJoint(4).Rz.Pos = -11.615576806649791;  % deg
smiData.RevoluteJoint(4).ID = "[Part2^Assem1-3:-:Part3-2]";

smiData.RevoluteJoint(5).Rz.Pos = 87.145052191703229;  % deg
smiData.RevoluteJoint(5).ID = "[Part3-2:-:Part5-2]";

