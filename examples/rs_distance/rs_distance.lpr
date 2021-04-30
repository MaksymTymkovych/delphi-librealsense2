program rs_distance;

{$MODE DELPHI}
{$APPTYPE CONSOLE}

uses
  librealsense2 in './../../src/librealsense2.pas',
  rsutil in './../../src/rsutil.pas';

const
  STREAM_DEPTH       = RS2_STREAM_DEPTH;
  FORMAT_DEPTH       = RS2_FORMAT_Z16;
  WIDTH        = 848;
  HEIGHT       = 480;
  FPS          = 30;
  STREAM_INDEX = 0;

  EXIT_FAILURE = 1;
  EXIT_SUCCESS = 0;

procedure check_error(e: Prs2_error);
begin
  if e <> nil then
  begin
    Writeln('rs_error was raised when calling ', rs2_get_failed_function(e),'(', rs2_get_failed_args(e) ,')');
    Writeln(AnsiString(rs2_get_error_message(e)));
    Halt(EXIT_FAILURE);
  end;
end;

procedure print_device_info(dev: Prs2_device);
var
  e: Prs2_error;
begin
  e := nil;
  Writeln('Using device 0, an ', rs2_get_device_info(dev, RS2_CAMERA_INFO_NAME, @e));
  check_error(e);
  Writeln('Serial number: ', rs2_get_device_info(dev, RS2_CAMERA_INFO_SERIAL_NUMBER, @e));
  check_error(e);
  Writeln('Firmware version: ', rs2_get_device_info(dev, RS2_CAMERA_INFO_FIRMWARE_VERSION, @e));
  check_error(e);
end;



var
  error: Prs2_error = nil;

  ctx: Trs2_context;
  device_list: Prs2_device_list;

  dev_count: Integer;
  dev: Prs2_device;
  pipeline: Prs2_pipeline;
  config: Prs2_config;
  pipeline_profile: Prs2_pipeline_profile;

  frames: Prs2_frame;

  num_of_frames: Integer;

  i: Integer;

  frame: Prs2_frame;

  w, h: Integer;

  dist_to_center: Single;
begin


  Writeln('Loaded: ', InitLibRealsense2());

  // Create a context object. This object owns the handles to all connected realsense devices.
  // The returned object should be released with rs2_delete_context(...)
  ctx := rs2_create_context(RS2_API_VERSION, @error);
  check_error(error);

  //* Get a list of all the connected devices. */
  // The returned object should be released with rs2_delete_device_list(...)
  device_list := rs2_query_devices(ctx, @error);
  check_error(error);

  dev_count := rs2_get_device_count(device_list, @error);
  check_error(error);
  Writeln('There are ', dev_count ,' connected RealSense devices.');
  if (0 = dev_count) then
    Halt(EXIT_FAILURE);

  // Get the first connected device
  // The returned object should be released with rs2_delete_device(...)
  dev := rs2_create_device(device_list, 0, @error);
  check_error(error);

  print_device_info(dev);

  // Create a pipeline to configure, start and stop camera streaming
  // The returned object should be released with rs2_delete_pipeline(...)
  pipeline :=  rs2_create_pipeline(ctx, @error);
  check_error(error);

  // Create a config instance, used to specify hardware configuration
  // The retunred object should be released with rs2_delete_config(...)
  config := rs2_create_config(@error);
  check_error(error);

  // Request a specific configuration
  rs2_config_enable_stream(config, STREAM_DEPTH, STREAM_INDEX, WIDTH, HEIGHT, FORMAT_DEPTH, FPS, @error);
  check_error(error);

  // Start the pipeline streaming
  // The retunred object should be released with rs2_delete_pipeline_profile(...)
  pipeline_profile := rs2_pipeline_start_with_config(pipeline, config, @error);

  check_error(error);
  if (error <> nil) then
  begin
    Writeln('The connected device doesn''t support depth streaming!');
    Halt(EXIT_FAILURE);
  end;

  while True do
  begin
    // This call waits until a new composite_frame is available
    // composite_frame holds a set of frames. It is used to prevent frame drops
    // The returned object should be released with rs2_release_frame(...)
    frames := rs2_pipeline_wait_for_frames(pipeline, RS2_DEFAULT_TIMEOUT, @error);
    check_error(error);

    // Returns the number of frames embedded within the composite frame
    num_of_frames := rs2_embedded_frames_count(frames, @error);
    check_error(error);

    for i:=0 to num_of_frames-1 do
    begin
      // The retunred object should be released with rs2_release_frame(...)
      frame := rs2_extract_frame(frames, i, @error);
      check_error(error);

      // Check if the given frame can be extended to depth frame interface
      // Accept only depth frames and skip other frames
      if (0 = rs2_is_frame_extendable_to(frame, RS2_EXTENSION_DEPTH_FRAME, @error)) then
      begin
        continue;
      end;

      // Get the depth frame's dimensions
      w := rs2_get_frame_width(frame, @error);
      check_error(error);
      h := rs2_get_frame_height(frame, @error);
      check_error(error);

      // Query the distance from the camera to the object in the center of the image
      dist_to_center := rs2_depth_frame_get_distance(frame, w div 2, h div 2, @error);
      check_error(error);

      // Print the distance
      Writeln('The camera is facing an object ', dist_to_center, ' meters away.');
      rs2_release_frame(frame);

    end;

    rs2_release_frame(frames);
  end;

  // Stop the pipeline streaming
  rs2_pipeline_stop(pipeline, @error);
  check_error(error);

  // Release resources
  rs2_delete_pipeline_profile(pipeline_profile);
  rs2_delete_config(config);
  rs2_delete_pipeline(pipeline);
  rs2_delete_device(dev);
  rs2_delete_device_list(device_list);
  rs2_delete_context(ctx);

  Halt(EXIT_SUCCESS);
end.

