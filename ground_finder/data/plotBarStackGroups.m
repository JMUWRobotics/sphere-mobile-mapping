function [] = plotBarStackGroups(stackData, groupLabels, ran = 0)
  %% Plot a set of stacked bars, but group them according to labels provided.
  %%
  %% Params:
  %%      stackData is a 3D matrix (i.e., stackData(i, j, k) => (Group, Stack, StackElement))
  %%      groupLabels is a CELL type (i.e., { 'a', 1 , 20, 'because' };)
  %%
  %% Copyright 2011 Evan Bollig (bollig at scs DOT fsu ANOTHERDOT edu
  %%
  %%
  NumGroupsPerAxis = size(stackData, 1);
  NumStacksPerGroup = size(stackData, 2);
  NumElemsPerStack = size(stackData, 3);
  % Count off the number of bins
  groupBins = 1:NumGroupsPerAxis;
  MaxGroupWidth = 0.65; % Fraction of 1. If 1, then we have all bars in groups touching
  groupOffset = MaxGroupWidth/NumStacksPerGroup;

  hold on;
  for i=1:NumStacksPerGroup
      Y = squeeze(stackData(:,i,:));
      % Center the bars:
      internalPosCount = i - ((NumStacksPerGroup+1) / 2);
      % Offset the group draw positions:
      groupDrawPos = (internalPosCount)* groupOffset + groupBins;

      h(i,:) = bar(Y, 'stacked');
      set(h(i,:),'BarWidth',groupOffset - 0.025);
      set(h(i,:),'XData',groupDrawPos);
  end
  % color TODO
  green = [0, 170, 105; 34, 182, 125; 68, 194, 145; 102, 206, 165; 137, 219, 185; 171, 231, 205; 205, 243, 225; 239, 255, 245];
  green = green / 255;
  blue = [0, 91, 171; 34, 113, 183; 69, 136, 195; 103, 158, 207; 137, 181, 219; 171, 203, 231; 206, 226, 243; 240, 248, 255];
  blue = blue / 255;
  red = [171, 0, 17; 183, 34, 49; 195, 69, 81; 207, 103, 113; 219, 137, 144; 231, 171, 176; 243, 206, 208; 255, 240, 240];
  red = red / 255;
  yellow = [214, 186, 0; 220, 195, 34; 226, 205, 69; 232, 214, 103; 237, 224, 137; 243, 233, 171; 249, 243, 206; 255, 252, 240];
  yellow = yellow / 255;
  magenta = [196, 0, 160; 204, 35, 173; 213, 71, 186; 221, 106, 199; 230, 142, 213; 238, 177, 226; 247, 213, 239; 255, 248, 252];
  magenta = magenta / 255;

  if ran==0
    if NumElemsPerStack == 8
      for i = 1:8
        set (h(1,i), "facecolor", red(i,:));
        set (h(2,i), "facecolor", yellow(i,:));
        set (h(3,i), "facecolor", green(i,:));
        set (h(4,i), "facecolor", blue(i,:));
        set (h(5,i), "facecolor", magenta(i,:));
      endfor
    elseif NumElemsPerStack == 4
      count = 2;
      for i = 1:4
        set (h(1,i), "facecolor", red(count,:));
        set (h(2,i), "facecolor", yellow(count,:));
        set (h(3,i), "facecolor", green(count,:));
        set (h(4,i), "facecolor", blue(count,:));
        set (h(5,i), "facecolor", magenta(count,:));
        count = count + 2;
      endfor
    endif
    hold off;
    set(gca,'XTickMode','manual');
    set(gca,'XTick',1:NumGroupsPerAxis);
    set(gca,'XTickLabelMode','manual');
    set(gca,'XTickLabel',groupLabels);
    ylabel("Mean Runtime [ms]");
    legend([h(1,1) h(2,1) h(3,1) h(4,1) h(5,1)], "LSF", "PCA", "RANSAC", "RHT", "RHT2");
   else
    if NumElemsPerStack == 8
      for i = 1:8
        set (h(1,i), "facecolor", red(i,:));
        set (h(2,i), "facecolor", blue(i,:));
      endfor
    elseif NumElemsPerStack == 4
      count = 2;
      for i = 1:4
        set (h(1,i), "facecolor", red(count,:));
        set (h(2,i), "facecolor", blue(count,:));
        count = count + 2;
      endfor
    endif
    hold off;
    set(gca,'XTickMode','manual');
    set(gca,'XTick',1:NumGroupsPerAxis);
    set(gca,'XTickLabelMode','manual');
    set(gca,'XTickLabel',groupLabels);
    ylabel("Mean Runtime [ms]");
    legend([h(1,1) h(2,1)], "RAN-PCA", "RAN-LSF");
  endif
end
