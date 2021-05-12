function f=rosen_20(x)
  if length(x) ~= 20 error('dimension must be 20'); end
  f = 100*sum((x(1:end-1).^2 - x(2:end)).^2) + sum((x(1:end-1)-1).^2);
end
