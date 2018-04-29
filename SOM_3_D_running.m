%% SOM 2-D
close all
clc
samples = 1000;
som_x = 5;
som_y = 5;
som_z = 5;
eta_i = 0.1;    %better results for eta_i = 0.5
sigma_i = 1;    %initial sigma
tau1 = 3000;
tau2 = 200;
h = 0;          %neighbourhood function

%% Generation of data for input space
input_space = rand(3,samples);


%% Initialization of SOM
weights = 0.8*rand(som_x,som_y,som_z,3)-1;


%% SOM Algo

scatter3(input_space(1,:),input_space(2,:),input_space(3,:),'.','linewidth',0.2);
hold on
weight_x = reshape(weights(:,:,:,1),[],1);
weight_y = reshape(weights(:,:,:,2),[],1);
weight_z = reshape(weights(:,:,:,3),[],1);
scatter3(weight_x,weight_y,weight_z,'bo');
hold off
pause(0.1)

for m = 1:300
    eta = eta_i*exp(-(m/tau2));
    display(m)
    display(eta)
    
    for i = 1:samples
        
        % Finding out winning neuron
        dist = zeros(som_x,som_y,som_z);
        for x = 1:som_x
            for y = 1:som_y
                for z = 1:som_z
                    w = [weights(x,y,z,1);weights(x,y,z,2);weights(x,y,z,3)];
                    dist(x,y,z) = norm(w-input_space(:,i));
                end
            end
        end
        [C,I] = min(dist(:));
        [I1,I2,I3] = ind2sub(size(dist),I);
        index_min = [I1;I2;I3];
        
        %weight updation
        sigma = sigma_i*exp(-(i/tau1));
        for x = 1:som_x
            for y = 1:som_y
                for z = 1:som_z
                    eucl_dist = norm([x;y;z]-index_min);
                    h = exp(-(eucl_dist^2/(2*sigma^2)));
                    weights(x,y,z,1) = weights(x,y,z,1) + eta*h*(input_space(1,i)-weights(x,y,z,1));
                    weights(x,y,z,2) = weights(x,y,z,2) + eta*h*(input_space(2,i)-weights(x,y,z,2));
                    weights(x,y,z,3) = weights(x,y,z,3) + eta*h*(input_space(3,i)-weights(x,y,z,3));
                end
            end
        end
        
        
    end
    scatter3(input_space(1,:),input_space(2,:),input_space(3,:),'.','linewidth',0.2);
    hold on
    weight_x = reshape(weights(:,:,:,1),[],1);
    weight_y = reshape(weights(:,:,:,2),[],1);
    weight_z = reshape(weights(:,:,:,3),[],1);
    scatter3(weight_x,weight_y,weight_z,'bo');
    hold off
    pause(0.1)
end

%save('SOM_eg_2_8_3_D');



