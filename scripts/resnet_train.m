function [resnet_mod,accuracy] = resnet_train
    % Load the images from the data store and divide the datastore into
    % three sets - [training set, test set, validation set]. Preprocess
    % the images to convert to 224X224.
    pds = imageDatastore('/tmp/test_images','IncludeSubfolders',true,'LabelSource','foldernames');
    pds.ReadFcn = @(loc)imresize(imread(loc),[224,224]);
    [traImgs,teImgs,valImgs] = splitEachLabel(pds,0.6,0.2,'randomize');
    traImgs.countEachLabel
    teImgs.countEachLabel
    valImgs.countEachLabel
    % Extract the classes from the datastore
    numClasses = numel(categories(pds.Labels))

    % Load the resnet neural network and plot the layers
    net = resnet101;
    lgraph = layerGraph(net);
    %figure('Units','normalized','Position',[0.1 0.1 0.8 0.8]);
    %plot(lgraph);

    % Remove the last three layers to change it to the required
    % classifications
    lgraph = removeLayers(lgraph, {'fc1000','prob','ClassificationLayer_predictions'});

    newLayers = [
        fullyConnectedLayer(numClasses,'Name','fc','WeightLearnRateFactor',20,'BiasLearnRateFactor', 20)
        softmaxLayer('Name','softmax')
        classificationLayer('Name','classoutput')];
    lgraph = addLayers(lgraph,newLayers);

    % Connect the new layers in the graph
    lgraph = connectLayers(lgraph,'pool5','fc');

    % Plot the new graph
    %figure('Units','normalized','Position',[0.3 0.3 0.4 0.4]);  
    %plot(lgraph)
    %ylim([0,10])

    % Set the training options for the new network
    options = trainingOptions('sgdm',...
        'MiniBatchSize',10,...
        'MaxEpochs',3,...
        'InitialLearnRate',1e-3,...
        'VerboseFrequency',1,...
        'ValidationData',valImgs,...
        'ValidationFrequency',3,...
        'Plots','training-progress');

    % Train the new network
    [resnet_mod,info] = trainNetwork(traImgs, lgraph, options);
    testpreds = classify(resnet_mod,teImgs);
    accuracy = nnz(testpreds == teImgs.Labels)/numel(testpreds)
    [placeconf,placenames] = confusionmat(teImgs.Labels,testpreds);
    heatmap(placenames,placenames,placeconf);
end