total_cost=0;
tic

parfor i=1:40
    
    total_cost=total_cost+test_rosen();
    
end

toc

average_cost=total_cost/40