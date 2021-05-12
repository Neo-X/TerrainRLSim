function xmin=purecmaes  
  % (mu/mu_w, lambda)-CMA-ES 
  % CMA-ES: Evolution Strategy with Covariance Matrix Adaptation for
  % nonlinear function minimization. To be used under the terms of the
  % GNU General Public License (http://www.gnu.org/copyleft/gpl.html).
  % Copyright: Nikolaus Hansen, 2003-09. 
  % e-mail: hansen[at]lri.fr
  %
  % This code is an excerpt from cmaes.m and implements the key parts
  % of the algorithm. It is intendend to be used for READING and
  % UNDERSTANDING the basic flow and all details of the CMA-ES
  % *algorithm*. Use the cmaes.m code to run serious simulations: it
  % is longer, but offers restarts, far better termination options, 
  % and supposedly quite useful output.
  %
  % URL: http://www.lri.fr/~hansen/purecmaes.m
  % References: See end of file. Last change: October, 21, 2010

  % --------------------  Initialization --------------------------------  
  % User defined input parameters (need to be edited)
  N = 50;               % number of objective variables/problem dimension
  
  task = task_pointmassmotion(N);
  
  xmean = rand(N,1);    % objective variables initial point
  sigma = 0.05;          % coordinate wise standard deviation (step size)
  stopfitness = 1e-10;  % stop if fitness < stopfitness (minimization)
  stopeval = 2000;   % stop after stopeval number of function evaluations
  
  % Strategy parameter setting: Selection  
  lambda = 1+floor(3*log(N));  % population size, offspring number
  mu = lambda/2;               % number of parents/points for recombination
  weights = log(mu+1/2)-log(1:mu)'; % muXone array for weighted recombination
  mu = floor(mu);        
  weights = weights/sum(weights);     % normalize recombination weights array
  mueff=sum(weights)^2/sum(weights.^2); % variance-effectiveness of sum w_i x_i

  % Strategy parameter setting: Adaptation
  cc = (4 + mueff/N) / (N+4 + 2*mueff/N); % time constant for cumulation for C
  cs = (mueff+2) / (N+mueff+5);  % t-const for cumulation for sigma control
  c1 = 2 / ((N+1.3)^2+mueff);    % learning rate for rank-one update of C
  cmu = min(1-c1, 2 * (mueff-2+1/mueff) / ((N+2)^2+mueff));  % and for rank-mu update
  damps = 1 + 2*max(0, sqrt((mueff-1)/(N+1))-1) + cs; % damping for sigma 
                                                      % usually close to 1
  % Initialize dynamic (internal) strategy parameters and constants
  pc = zeros(N,1); ps = zeros(N,1);   % evolution paths for C and sigma
  B = eye(N,N);                       % B defines the coordinate system
  D = ones(N,1);                      % diagonal D defines the scaling
  C = B * diag(D.^2) * B';            % covariance matrix C
  invsqrtC = B * diag(D.^-1) * B';    % C^-1/2 
  eigeneval = 0;                      % track update of B and D
  chiN=N^0.5*(1-1/(4*N)+1/(21*N^2));  % expectation of 
                                      %   ||N(0,I)|| == norm(randn(N,1))
  out.dat = []; out.datx = [];  % for plotting output
  learningData=zeros(1,2);

  % -------------------- Generation Loop --------------------------------
  counteval = 0;  % the next 40 lines contain the 20 lines of interesting code 
  while counteval < stopeval
    
    % Generate and evaluate lambda offspring
    for k=1:lambda,
      arx(:,k) = xmean + sigma * B * (D .* randn(N,1)); % m + sig * Normal(0,C) 
      %arfitness(k) = feval(strfitnessfct, arx(:,k)); % objective function call
      arfitness(k) = task.perform_rollout(task,arx(:,k)');%.*[0.0001 1 10 1 0.01]);
      counteval = counteval+1;
    end
    
    % Sort by fitness and compute weighted mean into xmean
    [arfitness, arindex] = sort(arfitness);  % minimization
    xold = xmean;
    xmean = arx(:,arindex(1:mu)) * weights;  % recombination, new mean value
    
    % Cumulation: Update evolution paths
    ps = (1-cs) * ps ... 
          + sqrt(cs*(2-cs)*mueff) * invsqrtC * (xmean-xold) / sigma; 
    hsig = sum(ps.^2)/(1-(1-cs)^(2*counteval/lambda))/N < 2 + 4/(N+1);
    pc = (1-cc) * pc ...
          + hsig * sqrt(cc*(2-cc)*mueff) * (xmean-xold) / sigma; 

    % Adapt covariance matrix C
    artmp = (1/sigma) * (arx(:,arindex(1:mu)) - repmat(xold,1,mu));  % mu difference vectors
    C = (1-c1-cmu) * C ...                   % regard old matrix  
         + c1 * (pc * pc' ...                % plus rank one update
                 + (1-hsig) * cc*(2-cc) * C) ... % minor correction if hsig==0
         + cmu * artmp * diag(weights) * artmp'; % plus rank mu update 

    % Adapt step size sigma
    sigma = sigma * exp((cs/damps)*(norm(ps)/chiN - 1)); 
    
    % Update B and D from C
    if counteval - eigeneval > lambda/(c1+cmu)/N/10  % to achieve O(N^2)
      eigeneval = counteval;
      C = triu(C) + triu(C,1)'; % enforce symmetry
      [B,D] = eig(C);           % eigen decomposition, B==normalized eigenvectors
      D = sqrt(diag(D));        % D contains standard deviations now
      invsqrtC = B * diag(D.^-1) * B';
    end
    
    % Break, if fitness is good enough or condition exceeds 1e14, better termination methods are advisable 
%     if arfitness(1) <= stopfitness || max(D) > 1e7 * min(D)
%       break;
%     end

    disp([num2str(counteval) ': ' num2str(arfitness(1)) ' ' ... 
          num2str(sigma*sqrt(max(diag(C)))) ' ' ...
          num2str(max(D) / min(D))]);

    % with long runs, the next line becomes time consuming
    out.dat = [out.dat; arfitness(1) sigma 1e5*D' ]; 
    out.datx = [out.datx; xmean'];
    %eig(C)
    learningData(end+1,:)=[counteval arfitness(1)];
  end % while, end generation loop

  % ------------- Final Message and Plotting Figures --------------------
  learningData(1,:)=[];
  learningData(end,:)=[];
figure
subplot(2,3,1)
plot(learningData(:,1),learningData(:,2))
title('Learning curve');
[y, yd, ydd,time]=task.perform_rollout_evaluation(task,xmean');
xlabel('number of roll-outs')
ylabel('cost')

subplot(2,3,2)
plot(time,y)
xlabel('time [s]')
ylabel('position [m]')
title('Trajectory');

subplot(2,3,3)
plot(time,yd)
xlabel('time [s]')
ylabel('velocity [m/s]')
title('Velocity');

subplot(2,3,4)
plot(time,ydd.*task.mass)
xlabel('time [s]')
ylabel('force [N]')
title('Force');
hold on
animation_handle=subplot(2,3,5:6);
title('Animation');
axis([-0.3 1.3 0 0.5])

for animation_time=1:length(time)
    pause(0.06)
    cla(animation_handle)
    box=rectangle('Position',[y(animation_time),0,0.1,0.1]);
    xlabel('position [m]')
end

  
% ---------------------------------------------------------------  
function f=frosenbrock(x)
  if size(x,1) < 2 error('dimension must be greater one'); end
  f = 100*sum((x(1:end-1).^2 - x(2:end)).^2) + sum((x(1:end-1)-1).^2);

function f=fsphere(x)
  f=sum(x.^2);
  
function f=fssphere(x)
  f=sqrt(sum(x.^2));
  
function f=fschwefel(x)
  f = 0;
  for i = 1:size(x,1),
    f = f+sum(x(1:i))^2;
  end

function f=fcigar(x)
  f = x(1)^2 + 1e6*sum(x(2:end).^2);
  
function f=fcigtab(x)
  f = x(1)^2 + 1e8*x(end)^2 + 1e4*sum(x(2:(end-1)).^2);
  
function f=ftablet(x)
  f = 1e6*x(1)^2 + sum(x(2:end).^2);
  
function f=felli(x)
  N = size(x,1); if N < 2 error('dimension must be greater one'); end
  f=1e6.^((0:N-1)/(N-1)) * x.^2;

function f=felli100(x)
  N = size(x,1); if N < 2 error('dimension must be greater one'); end
  f=1e4.^((0:N-1)/(N-1)) * x.^2;

function f=fplane(x)
  f=x(1);

function f=ftwoaxes(x)
  f = sum(x(1:floor(end/2)).^2) + 1e6*sum(x(floor(1+end/2):end).^2);

function f=fparabR(x)
  f = -x(1) + 100*sum(x(2:end).^2);

function f=fsharpR(x)
  f = -x(1) + 100*norm(x(2:end));
  
function f=fdiffpow(x)
  N = size(x,1); if N < 2 error('dimension must be greater one'); end
  f=sum(abs(x).^(2+10*(0:N-1)'/(N-1)));
  
function f=frastrigin10(x)
  N = size(x,1); if N < 2 error('dimension must be greater one'); end
  scale=10.^((0:N-1)'/(N-1));
  f = 10*size(x,1) + sum((scale.*x).^2 - 10*cos(2*pi*(scale.*x)));

function f=frand(x)
  f=rand;

% ---------------------------------------------------------------  
%%% REFERENCES
%
% Hansen, N. and S. Kern (2004). Evaluating the CMA Evolution
% Strategy on Multimodal Test Functions.  Eighth International
% Conference on Parallel Problem Solving from Nature PPSN VIII,
% Proceedings, pp. 282-291, Berlin: Springer. 
% (http://www.bionik.tu-berlin.de/user/niko/ppsn2004hansenkern.pdf)
% 
% Further references:
% Hansen, N. and A. Ostermeier (2001). Completely Derandomized
% Self-Adaptation in Evolution Strategies. Evolutionary Computation,
% 9(2), pp. 159-195.
% (http://www.bionik.tu-berlin.de/user/niko/cmaartic.pdf).
%
% Hansen, N., S.D. Mueller and P. Koumoutsakos (2003). Reducing the
% Time Complexity of the Derandomized Evolution Strategy with
% Covariance Matrix Adaptation (CMA-ES). Evolutionary Computation,
% 11(1).  (http://mitpress.mit.edu/journals/pdf/evco_11_1_1_0.pdf).
%
