function cam = get_webcam(cam_num)
%GET_WEBCAM gets a webcam with proper setup
%   Sets ExposureMode and WhiteBalanceMode to manual
    cam = webcam(cam_num);
    cam.ExposureMode = 'manual';
    cam.WhiteBalanceMode = 'manual';
    cam.Exposure = -7;
    cam.WhiteBalance = 2800;
end