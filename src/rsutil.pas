(*
   License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2015 Intel Corporation. All Rights Reserved.
 *)
unit rsutil;

{$MODE delphi}
{$INCLUDE 'librealsense2.inc'}

interface

uses
  librealsense2,
  SysUtils,
  Math;

type
  Prs2_pixelf = ^Trs2_pixelf;
  Trs2_pixelf = Array [0..1] of Single;

  Prs2_pointf = ^Trs2_pointf;
  Trs2_pointf = Array [0..2] of Single;

const
  FLT_EPSILON = 1E-5;

implementation

(*
   Given a point in 3D space, compute the corresponding pixel coordinates in an
   image with no distortion or forward distortion coefficients
   produced by the same camera
 *)
procedure rs2_project_point_to_pixel(var pixel: Trs2_pixelf;
                                     const intrin: Prs2_intrinsics;
                                     const point: Trs2_pointf
                                     //float pixel[2], const struct rs2_intrinsics * intrin, const float point[3]
                                     );
var
  x, y: Single;
  r2, f: Single;
  dx, dy: Single;

  xf, yf: Single;
  r, rd: Single;
  theta, theta2, series: Single;
begin
  x := point[0] / point[2]; y := point[1] / point[2];

  if ((intrin.model = RS2_DISTORTION_MODIFIED_BROWN_CONRADY) or
      (intrin.model = RS2_DISTORTION_INVERSE_BROWN_CONRADY)) then
  begin
    r2 := x*x + y*y;
    f := 1 + intrin.coeffs[0]*r2 + intrin.coeffs[1]*r2*r2 + intrin.coeffs[4]*r2*r2*r2;
    x := x * f;
    y := y * f;
    dx := x + 2*intrin.coeffs[2]*x*y + intrin.coeffs[3]*(r2 + 2*x*x);
    dy := y + 2*intrin.coeffs[3]*x*y + intrin.coeffs[2]*(r2 + 2*y*y);
    x := dx;
    y := dy;
  end;

  if intrin.model = RS2_DISTORTION_BROWN_CONRADY then
  begin
    r2 := x * x + y * y;
    f := 1 + intrin.coeffs[0] * r2 + intrin.coeffs[1] * r2*r2 + intrin.coeffs[4] * r2*r2*r2;

    xf := x * f;
    yf := y * f;

    dx := xf + 2 * intrin.coeffs[2] * x*y + intrin.coeffs[3] * (r2 + 2 * x*x);
    dy := yf + 2 * intrin.coeffs[3] * x*y + intrin.coeffs[2] * (r2 + 2 * y*y);

    x := dx;
    y := dy;
  end;

  if intrin.model = RS2_DISTORTION_FTHETA then
  begin
    r := sqrt(x*x + y*y);
    if r < FLT_EPSILON then
    begin
      r := FLT_EPSILON;
    end;
    rd := (1.0 / intrin.coeffs[0] * ArcTan(2 * r* tan(intrin.coeffs[0] / 2.0)));
    x := x * rd / r;
    y := y * rd / r;
  end;

  if intrin.model = RS2_DISTORTION_KANNALA_BRANDT4 then
  begin
    r := sqrt(x*x + y*y);
    if r < FLT_EPSILON then
    begin
      r := FLT_EPSILON;
    end;
    theta := ArcTan(r);
    theta2 := theta*theta;
    series := 1 + theta2*(intrin.coeffs[0] + theta2*(intrin.coeffs[1] + theta2*(intrin.coeffs[2] + theta2*intrin.coeffs[3])));
    rd := theta*series;
    x := x * rd / r;
    y := y * rd / r;
  end;

  pixel[0] := x * intrin.fx + intrin.ppx;
  pixel[1] := y * intrin.fy + intrin.ppy;
end;

(*
   Given pixel coordinates and depth in an image with no distortion or inverse
   distortion coefficients, compute the corresponding point in 3D space
   relative to the same camera
 *)
procedure rs2_deproject_pixel_to_point(point: Trs2_pointf;
                                       const intrin: Prs2_intrinsics;
                                       const pixel: Trs2_pixelf;
                                       depth: Single);
var
  x, y: Single;
  r2, r, ux, uy, f: Single;
  rd: Single;
  theta, theta2: Single;
  i: Integer;
  df: Single;
begin
  assert(intrin.model <> RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
  assert(intrin.model <> RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model

  x := (pixel[0] - intrin.ppx) / intrin.fx;
  y := (pixel[1] - intrin.ppy) / intrin.fy;
  if intrin.model = RS2_DISTORTION_INVERSE_BROWN_CONRADY then
  begin
    r2 := x*x + y*y;
    f := 1 + intrin.coeffs[0]*r2 + intrin.coeffs[1]*r2*r2 + intrin.coeffs[4]*r2*r2*r2;
    ux := x*f + 2*intrin.coeffs[2]*x*y + intrin.coeffs[3]*(r2 + 2*x*x);
    uy := y*f + 2*intrin.coeffs[3]*x*y + intrin.coeffs[2]*(r2 + 2*y*y);
    x := ux;
    y := uy;
  end;
  if intrin.model = RS2_DISTORTION_KANNALA_BRANDT4 then
  begin
    rd := sqrt(x*x + y*y);
    if rd < FLT_EPSILON then
    begin
      rd := FLT_EPSILON;
    end;

    theta := rd;
    theta2 := rd*rd;
    for i:=0 to 4-1 do
    begin
      f := theta*(1 + theta2*(intrin.coeffs[0] + theta2*(intrin.coeffs[1] + theta2*(intrin.coeffs[2] + theta2*intrin.coeffs[3])))) - rd;
      if abs(f) < FLT_EPSILON then
      begin
        break;
      end;
      df := 1 + theta2*(3 * intrin.coeffs[0] + theta2*(5 * intrin.coeffs[1] + theta2*(7 * intrin.coeffs[2] + 9 * theta2*intrin.coeffs[3])));
      theta := theta - f / df;
      theta2 := theta*theta;
    end;
    r := tan(theta);
    x := x * r / rd;
    y := y * r / rd;
  end;
  if intrin.model = RS2_DISTORTION_FTHETA then
  begin
    rd := sqrt(x*x + y*y);
    if rd < FLT_EPSILON then
    begin
      rd := FLT_EPSILON;
    end;
    r := (tan(intrin.coeffs[0] * rd) / ArcTan(2 * tan(intrin.coeffs[0] / 2.0)));
    x := x * r / rd;
    y := y * r / rd;
  end;

  point[0] := depth * x;
  point[1] := depth * y;
  point[2] := depth;
end;

(*
   Transform 3D coordinates relative to one sensor to 3D coordinates relative
   to another viewpoint
 *)
procedure rs2_transform_point_to_point(to_point: Prs2_pointf;
                                       const extrin: Prs2_extrinsics;
                                       const from_point: Prs2_pointf);
begin
    to_point[0] := extrin.rotation[0] * from_point[0] + extrin.rotation[3] * from_point[1] + extrin.rotation[6] * from_point[2] + extrin.translation[0];
    to_point[1] := extrin.rotation[1] * from_point[0] + extrin.rotation[4] * from_point[1] + extrin.rotation[7] * from_point[2] + extrin.translation[1];
    to_point[2] := extrin.rotation[2] * from_point[0] + extrin.rotation[5] * from_point[1] + extrin.rotation[8] * from_point[2] + extrin.translation[2];
end;

(* Calculate horizontal and vertical feild of view, based on video intrinsics *)
procedure rs2_fov(const intrin: Prs2_intrinsics;
                  to_fov: Prs2_pixelf);
begin
    to_fov[0] := (ArcTan2(intrin.ppx + 0.5, intrin.fx) + ArcTan2(intrin.width - (intrin.ppx + 0.5), intrin.fx)) * 57.2957795;
    to_fov[1] := (ArcTan2(intrin.ppy + 0.5, intrin.fy) + ArcTan2(intrin.height - (intrin.ppy + 0.5), intrin.fy)) * 57.2957795;
end;

procedure next_pixel_in_line(curr: Trs2_pixelf;
                             const start: Trs2_pixelf;
                             const end_: Trs2_pixelf);
var
  line_slope: Single;
begin
  line_slope := (end_[1] - start[1]) / (end_[0] - start[0]);
  if Abs(end_[0] - curr[0]) > Abs(end_[1] - curr[1]) then
  begin
    if end_[0] > curr[0] then
      curr[0] := curr[0] + 1
    else
      curr[0] := curr[0] - 1;

    curr[1] := end_[1] - line_slope * (end_[0] - curr[0]);
  end
  else
  begin
    if end_[1] > curr[1] then
      curr[1] := curr[1] + 1
    else
      curr[1] := curr[1] - 1;
    curr[0] := end_[0] - ((end_[1] + curr[1]) / line_slope);
  end;
end;

function is_pixel_in_line(const curr: Trs2_pixelf;
                          const start: Trs2_pixelf;
                          const end_: Trs2_pixelf): Boolean;
begin
  Result := (((end_[0] >= start[0]) and (end_[0] >= curr[0]) and (curr[0] >= start[0])) or ((end_[0] <= start[0]) and (end_[0] <= curr[0]) and (curr[0] <= start[0]))) and
            (((end_[1] >= start[1]) and (end_[1] >= curr[1]) and (curr[1] >= start[1])) or ((end_[1] <= start[1]) and (end_[1] <= curr[1]) and (curr[1] <= start[1])));
end;

procedure adjust_2D_point_to_boundary(p: Trs2_pixelf;
                                      width, height: Integer);
begin
    if p[0] < 0 then p[0] := 0;
    if p[0] > width then p[0] := width;
    if p[1] < 0 then p[1] := 0;
    if p[1] > height then p[1] := height;
end;

(* Find projected pixel with unknown depth search along line. *)
procedure rs2_project_color_pixel_to_depth_pixel(to_pixel: Trs2_pixelf;
                                                 const data: PWord;
                                                 depth_scale: Single;
                                                 depth_min: Single;
                                                 depth_max: Single;
                                                 const depth_intrin: Prs2_intrinsics;
                                                 const color_intrin: Prs2_intrinsics;
                                                 const color_to_depth: Prs2_extrinsics;
                                                 const depth_to_color: Prs2_extrinsics;
                                                 const from_pixel: Trs2_pixelf);
var
  min_dist: Single;
  start_pixel, end_pixel, projected_pixel, p: Trs2_pixelf;
  min_point, min_transformed_point,
  max_point, max_transformed_point,
  point, transformed_point: Trs2_pointf;
  depth, new_dist : Single;
begin
    //Find line start pixel
    start_pixel[0] := 0; start_pixel[1] := 0;
    min_point[0] := 0; min_point[1] := 0; min_point[2] := 0;
    min_transformed_point[0] := 0; min_transformed_point[1] := 0; min_transformed_point[2] := 0;
    rs2_deproject_pixel_to_point(min_point, color_intrin, from_pixel, depth_min);
    rs2_transform_point_to_point(@min_transformed_point, @color_to_depth, @min_point);
    rs2_project_point_to_pixel(start_pixel, depth_intrin, min_transformed_point);
    adjust_2D_point_to_boundary(start_pixel, depth_intrin.width, depth_intrin.height);

    //Find line end depth pixel
    end_pixel[0] := 0; end_pixel[1] := 0;
    max_point[0] := 0; max_point[1] := 0; max_point[2] := 0;
    max_transformed_point[0] := 0; max_transformed_point[1] := 0; max_transformed_point[2] := 0;
    rs2_deproject_pixel_to_point(max_point, color_intrin, from_pixel, depth_max);
    rs2_transform_point_to_point(@max_transformed_point, @color_to_depth, @max_point);
    rs2_project_point_to_pixel(end_pixel, depth_intrin, max_transformed_point);
    adjust_2D_point_to_boundary(end_pixel, depth_intrin.width, depth_intrin.height);

    //search along line for the depth pixel that it's projected pixel is the closest to the input pixel
    min_dist := -1;

    p[0] := start_pixel[0]; p[1] := start_pixel[1];
    //for (float p[2] = { start_pixel[0], start_pixel[1] }; is_pixel_in_line(p, start_pixel, end_pixel); next_pixel_in_line(p, start_pixel, end_pixel))
    while is_pixel_in_line(p, start_pixel, end_pixel) do
    begin
        depth := depth_scale * PWord(Integer(data) + ((Integer(p[1]) * depth_intrin.width + Integer(p[0])) << 1))^;
        if depth = 0 then
            continue;

        projected_pixel[0] := 0; projected_pixel[1] := 0;
        point[0] := 0; point[1] := 0; point[2] := 0;
        transformed_point[0] := 0; transformed_point[1] := 0; transformed_point[2] := 0;
        rs2_deproject_pixel_to_point(point, depth_intrin, p, depth);
        rs2_transform_point_to_point(@transformed_point, @depth_to_color, @point);
        rs2_project_point_to_pixel(projected_pixel, color_intrin, transformed_point);

        new_dist := Power((projected_pixel[1] - from_pixel[1]), 2) + Power((projected_pixel[0] - from_pixel[0]), 2);
        if (new_dist < min_dist) or (min_dist < 0) then
        begin
            min_dist := new_dist;
            to_pixel[0] := p[0];
            to_pixel[1] := p[1];
        end;
        next_pixel_in_line(p, start_pixel, end_pixel);
    end;
end;


end.

