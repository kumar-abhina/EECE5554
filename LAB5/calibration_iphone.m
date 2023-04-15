% Calibration results after optimization (with uncertainties):
% Focal Length:          fc = [ 3309.28735   3307.88781 ] +/- [ 5.67819   6.04517 ]
% Principal point:       cc = [ 2003.38344   1491.61805 ] +/- [ 8.51567   7.54095 ]
% Skew:             alpha_c = [ 0.00000 ] +/- [ 0.00000  ]   => angle of pixel axes = 90.00000 +/- 0.00000 degrees
% Distortion:            kc = [ 0.06506   -0.15297   -0.00108   -0.00105  0.00000 ] +/- [ 0.00630   0.01630   0.00080   0.00088  0.00000 ]
% Pixel error:          err = [ 1.73024   1.76844 ]

OgDir = fullfile("Calibration Images\"); %fullfile(toolboxdir('vision'),'visiondata','building');
ogscene = imageDatastore(OgDir);

corDir = fullfile("Calibration Images\Calibration Op\"); %fullfile(toolboxdir('vision'),'visiondata','building');
corscene = imageDatastore(corDir);

for i = 1:20
    I = readimage(ogscene,i);
    I2 = readimage(corscene, i);
    figure
    subplot(1,2,1), imshow(I)
    subplot(1,2,2), imshow(I2)
end