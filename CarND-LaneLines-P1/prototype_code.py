def draw_lines_fitline(img, lines, color=[255, 0, 0], thickness=2):
    """
    NOTE: this is the function you might want to use as a starting point once you want to 
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).  
    
    Think about things like separating line segments by their 
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of 
    the lines and extrapolate to the top and bottom of the lane.
    
    This function draws `lines` with `color` and `thickness`.    
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
    
    global mLeft, mRight
    
    leftLaneLines = np.array([]).reshape(0,2);
    rightLaneLines = np.array([]).reshape(0,2);
    minY = img.shape[0];
    for line in lines:
        for x1,y1,x2,y2 in line:
            slope = ((y1-y2)/(x2-x1));
            if slope < 0:
                rightLaneLines = np.vstack([rightLaneLines, [x1,y1],[x2,y2]]);
            else:
                leftLaneLines = np.vstack([leftLaneLines, [x1,y1],[x2,y2]]);
                
            minY = np.amin([minY, y1, y2])
#             cv2.line(img, (x1, y1), (x2, y2), color, thickness)
    
    # Left lane calculation

    if leftLaneLines.size != 0:
        leftLine = cv2.fitLine(leftLaneLines, cv2.DIST_L2, 0, 0.01, 0.01);
    
        vx,vy, x, y = leftLine;
        m = vy/vx;
        
        y1 = img.shape[0];
        x1 = int(( (y1-y) + (m * x) ) / m);

        y2 = minY;
        x2 = int( ((y1-y2) - (m * x1)) / (-m));
        cv2.line(img, (x1, y1), (x2,y2), color, thickness);
    
    # Right lane calculation
    
    if rightLaneLines.size != 0:
    
        rightLine = cv2.fitLine(rightLaneLines, cv2.DIST_L12, 0, 0.01, 0.01);

        vx,vy, x, y = rightLine;
        m = vy/vx;
           
        y1 = img.shape[0];
        x1 = ( (y-y1) - (m * x) ) / (-m);

        y2 = minY;
        x2 = int( ((y2-y1) + (m * x1)) / (m));

        cv2.line(img, (x1, y1), (x2,y2), color, thickness);
