
function [J] = calcJ(pixels, b, f)
%%
  try    
    xl = pixels.left.x;
    yl = pixels.left.y;
    xr = pixels.right.x;
    yr = pixels.right.y;
  catch 
  end

  try 
    xl = pixels.left(1);
    yl = pixels.left(2);
    xr = pixels.right(1);
    yr = pixels.right(2);
  catch
  end
  
  try 
    xl = pixels.pixels_left(1);
    yl = pixels.pixels_left(2);
    xr = pixels.pixels_right(1);
    yr = pixels.pixels_right(2);
  catch
  end
  
  d = xl-xr;
  dd = d*d;
  
  %row 1
    J(1,1) = (-b * xr) / (2*dd);  
    J(1,2) = ( b * xl) / (2*dd); 
    J(1,3) =   0;
  %row 2
    J(2,1) = (-b * yl )/ dd;      
    J(2,2) = ( b * yl) / dd;     
    J(2,3) =   b/d;
  %row 3
    J(3,1) = (-b * f)  / dd;      
    J(3,2) = ( b * f)  / dd;     
    J(3,3) =   0;
  
end