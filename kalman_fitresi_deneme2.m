clear all; clc; close all;
bc=ones(1,2);
obj =imaq.VideoDevice('winvideo', 1, 'YUY2_640x480','ROI', [1 1 640 480]);
set(obj,'ReturnedColorSpace', 'rgb');
figure('menubar','none','tag','webcam');
wait=0;
kalmanFilter = configureKalmanFilter('ConstantVelocity', [240, 180], [1 1]*1e5, [10,10], 15);
foregroundDetector = vision.ForegroundDetector('NumTrainingFrames', 10, 'InitialVariance', 0.0005);
blobAnalyzer = vision.BlobAnalysis('AreaOutputPort', false,'MinimumBlobArea', 70, 'CentroidOutputPort', true, 'BoundingBoxOutputPort', true);
f = 1;
while (wait<50)
    wait=wait+1;
    [~, predictedLocation, P]= predict(kalmanFilter);
    predictedLocation = predictedLocation(1:2:4);
    wait=wait+1;
    frame=step(obj);
    grayImage = rgb2gray(frame);
    mask = step(foregroundDetector,grayImage);
    [~, bbox] = step(blobAnalyzer, mask);
    
    if(~isempty(bbox))
        centx=bbox(1) + (bbox(3)/2) ;
        centy=bbox(2) + (bbox(4)/2) ;
        
        bc=[centx,centy];
    end
        
    if numel(bbox(:)) ~= 0
         [~, smooth_bc, P] = correct(kalmanFilter, bc);
         smooth_bc = smooth_bc(1:2:4);
    else
        smooth_bc = predictedLocation;
    end
    boxInserter  = vision.ShapeInserter('BorderColor','Custom','CustomBorderColor',[255 0 0]);
    boxInserterBelief  = vision.ShapeInserter('BorderColor','Custom','CustomBorderColor',[0 0 255]);
    videoOut = step(boxInserter, frame, bbox);
     belief = [smooth_bc(1)-P(1,1)/2, smooth_bc(2)-P(3,3)/2, P(1,1), P(3,3)];
    videoOut = step(boxInserterBelief, videoOut, belief);
    
    videoOut = insertMarker(videoOut, smooth_bc); 
    imshow(videoOut,'border','tight');
    saveas(gcf, ['output_face', num2str(f), '.png']);
    f = f+1;
    release(obj);
end
        
