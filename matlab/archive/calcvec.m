function [vector] = calcvec(pixels, b, f, Cx)
   
  xl = pixels.pixels_left(1);
  yl = pixels.pixels_left(2);
  xr = pixels.pixels_right(1);
  yr = pixels.pixels_right(2);

d = (xl+Cx) - (xr+Cx);
vector(4,1) = 1;
vector(3,1) = b*f ./ d;
vector(2,1) = vector(3,1) .*(yr+yl) /(2*f);
vector(1,1) = vector(3,1) .*(xr+xl) /(2*f);


end
