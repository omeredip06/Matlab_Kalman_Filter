vid=videoinput('winvideo',1,'YUY2_320x240');

set(vid,'FramesPerTrigger',Inf);
set(vid,'ReturnedColorspace','rgb');
vid.FrameGrabInterval=5;

start(vid);

while(vid.FramesAcquired<=200)
    data=getsnapshot(vid);
    diff_im=imsubtract(data(:,:,3),rgb2gray(data));
    diff_im=medfilt2(diff_im,[3,3]);
    diff_im=im2bw(diff_im,0.18);
    diff_im=bwareaopen(diff_im,300);
    bw=bwlabel(diff_im,8);
    
    stats=regionprops(bw,'BoundingBox','Centroid');
    imshow(data);
    
    hold on
    
    for object=1:length(stats)
        bb=stats(object).BoundingBox;
        bc=stats(object).Centroid;
        
        rectangle('Position',bb,'EdgeColor','r','LineWidth',2);
        plot(bc(1),bc(2),'-m+')
    end
    
    hold off
end

stop(vid);
flushdata(vid);

clear all;
sprintf('%s','Bitti');

        