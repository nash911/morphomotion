%% Machine Learning Online Class
%  Exercise 1: Linear regression with multiple variables
%
%  Instructions
%  ------------
% 
%  This file contains code that helps you get started on the
%  linear regression exercise. 
%
%  You will need to complete the following functions in this 
%  exericse:
%
%     warmUpExercise.m
%     plotData.m
%     gradientDescent.m
%     computeCost.m
%     gradientDescentMulti.m
%     computeCostMulti.m
%     featureNormalize.m
%     normalEqn.m
%
%  For this part of the exercise, you will need to change some
%  parts of the code below for various experiments (e.g., changing
%  learning rates).
%

%% Initialization

%% ================ Part 1: Feature Normalization ================

%% Clear and Close Figures
clear all; close all; clc

fprintf('Loading data ...\n');

%% Load Data
%data = load('ex1data1.txt');
data = load('servo_data.csv');
%X = data(:, 1:2);
%y = data(:, 3);
X = data(:, 1);
y = data(:, 2);
m = length(y);
size(X)

% Print out some data points
fprintf('First 10 examples from the dataset: \n');
%fprintf(' x = [%.0f %.0f], y = %.0f \n', [X(1:10,:) y(1:10,:)]');
fprintf(' x = [%.0f], y = %.0f \n', [X(1:10,:) y(1:10,:)]');

fprintf('Program paused. Press enter to continue.\n');
pause;

% Scale features and set them to zero mean
fprintf('Normalizing Features ...\n');

[X mu sigma] = featureNormalize(X);

% Add intercept term to X
X = [ones(m, 1) X];


%% ================ Part 2: Gradient Descent ================

% ====================== YOUR CODE HERE ======================
% Instructions: We have provided you with the following starter
%               code that runs gradient descent with a particular
%               learning rate (alpha). 
%
%               Your task is to first make sure that your functions - 
%               computeCost and gradientDescent already work with 
%               this starter code and support multiple variables.
%
%               After that, try running gradient descent with 
%               different values of alpha and see which one gives
%               you the best result.
%
%               Finally, you should complete the code at the end
%               to predict the price of a 1650 sq-ft, 3 br house.
%
% Hint: By using the 'hold on' command, you can plot multiple
%       graphs on the same figure.
%
% Hint: At prediction, make sure you do the same feature normalization.
%

fprintf('Running gradient descent ...\n');

% Choose some alpha value
alpha = 1.0;
num_iters = 50;

% Init Theta and Run Gradient Descent 
%theta = zeros(3, 1);
theta = zeros(2, 1);
[theta, J_history] = gradientDescentMulti(X, y, theta, alpha, num_iters);

% Plot the convergence graph
figure;
plot(1:numel(J_history), J_history, '-b', 'LineWidth', 2);
xlabel('Number of iterations');
ylabel('Cost J');

% Display gradient descent's result
fprintf('Theta computed from gradient descent: \n');
fprintf(' %f \n', theta);
fprintf('\n');

% Estimate the price of a 1650 sq-ft, 3 br house
% ====================== YOUR CODE HERE ======================
% Recall that the first column of X is all-ones. Thus, it does
% not need to be normalized.

price = [1, (1650-mu(1))/sigma(1)] * theta; % 1650 sq-ft and 3 bedrooms
fprintf(['\n Predicted price of a 1650 sq-ft, 3 br house ' ...
         '(using gradient descent):\n $%f\n\n'], price);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Servo_1 cross validation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
price = [1, (76-mu(1))/sigma(1)] * theta; % 3º
fprintf(['Predicted angle at analog reading 76 is: %f While the real angle is 3º. \n'], price);

price = [1, (112-mu(1))/sigma(1)] * theta; % 26º
fprintf(['Predicted angle at analog reading 112 is: %f While the real angle is 26º. \n'], price);

price = [1, (123-mu(1))/sigma(1)] * theta; % 33º
fprintf(['Predicted angle at analog reading 123 is: %f While the real angle is 33º. \n'], price);

price = [1, (137-mu(1))/sigma(1)] * theta; % 42º
fprintf(['Predicted angle at analog reading 137 is: %f While the real angle is 42º. \n'], price);

price = [1, (151-mu(1))/sigma(1)] * theta; % 51º
fprintf(['Predicted angle at analog reading 151 is: %f While the real angle is 51º. \n'], price);

price = [1, (199-mu(1))/sigma(1)] * theta; % 82º
fprintf(['Predicted angle at analog reading 199 is: %f While the real angle is 82º. \n'], price);

price = [1, (225-mu(1))/sigma(1)] * theta; % 99º
fprintf(['Predicted angle at analog reading 225 is: %f While the real angle is 99º. \n'], price);

price = [1, (250-mu(1))/sigma(1)] * theta; % 115º
fprintf(['Predicted angle at analog reading 250 is: %f While the real angle is 115º. \n'], price);

price = [1, (271-mu(1))/sigma(1)] * theta; % 128º
fprintf(['Predicted angle at analog reading 271 is: %f While the real angle is 128º. \n'], price);

price = [1, (290-mu(1))/sigma(1)] * theta; % 140º
fprintf(['Predicted angle at analog reading 290 is: %f While the real angle is 140º. \n'], price);

price = [1, (309-mu(1))/sigma(1)] * theta; % 153º
fprintf(['Predicted angle at analog reading 309 is: %f While the real angle is 153º. \n'], price);

price = [1, (330-mu(1))/sigma(1)] * theta; % 166º
fprintf(['Predicted angle at analog reading 330 is: %f While the real angle is 166º. \n\n'], price);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Servo_1 cross validation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%price = [1, (375-mu(1))/sigma(1), (140625-mu(2))/sigma(2)] * theta; % 175º
%fprintf(['Predicted angle at analog reading 375 is: %f While the real angle is 175º. \n'], price);


% ============================================================

fprintf('Program paused. Press enter to continue.\n');
pause;

%% ================ Part 3: Normal Equations ================

fprintf('Solving with normal equations...\n');

% ====================== YOUR CODE HERE ======================
% Instructions: The following code computes the closed form 
%               solution for linear regression using the normal
%               equations. You should complete the code in 
%               normalEqn.m
%
%               After doing so, you should complete this code 
%               to predict the price of a 1650 sq-ft, 3 br house.
%

%% Load Data
data = load('servo_data.csv');
X = data(:, 1);
y = data(:, 2);
m = length(y);

% Add intercept term to X
X = [ones(m, 1) X];

% Calculate the parameters from the normal equation
theta = normalEqn(X, y);

% Display normal equation's result
fprintf('Theta computed from the normal equations: \n');
fprintf(' %f \n', theta);
fprintf('\n');


% Estimate the price of a 1650 sq-ft, 3 br house
% ====================== YOUR CODE HERE ======================
price = [1, 1650] * theta; % You should change this
fprintf(['Predicted price of a 1650 sq-ft, 3 br house ' ...
         '(using normal equations):\n $%f\n'], price);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Servo_1 cross validation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
price = [1, 70] * theta; % 3º
fprintf(['Predicted angle at analog reading 70 is: %f While the real angle is 3º. \n'], price);

price = [1, 104] * theta; % 26º
fprintf(['Predicted angle at analog reading 104 is: %f While the real angle is 26º. \n'], price);

price = [1, 114] * theta; % 33º
fprintf(['Predicted angle at analog reading 114 is: %f While the real angle is 33º. \n'], price);

price = [1, 127] * theta; % 42º
fprintf(['Predicted angle at analog reading 127 is: %f While the real angle is 42º. \n'], price);

price = [1, 140] * theta; % 51º
fprintf(['Predicted angle at analog reading 140 is: %f While the real angle is 51º. \n'], price);

price = [1, 186] * theta; % 82º
fprintf(['Predicted angle at analog reading 186 is: %f While the real angle is 82º. \n'], price);

price = [1, 210] * theta; % 99º
fprintf(['Predicted angle at analog reading 210 is: %f While the real angle is 99º. \n'], price);

price = [1, 233] * theta; % 115º
fprintf(['Predicted angle at analog reading 233 is: %f While the real angle is 115º. \n'], price);

price = [1, 252] * theta; % 128º
fprintf(['Predicted angle at analog reading 252 is: %f While the real angle is 128º. \n'], price);

price = [1, 269] * theta; % 140º
fprintf(['Predicted angle at analog reading 269 is: %f While the real angle is 140º. \n'], price);

price = [1, 288] * theta; % 153º
fprintf(['Predicted angle at analog reading 288 is: %f While the real angle is 153º. \n'], price);

price = [1, 307] * theta; % 166º
fprintf(['Predicted angle at analog reading 307 is: %f While the real angle is 166º. \n\n'], price);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Servo_1 cross validation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




% ============================================================

