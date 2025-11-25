%% Runtime evaluation
clear;

% ----------------------- kd_filter -----------------------
disp("");
disp("------------ KD FILTER ------------");
% file 1
csv1 = csvread("ikd/kd-filter000.csv");
runs1 = rows(csv1) - 2;
avg_csv1 = sum(csv1(2:end-1,:)/1000) / runs1;
avg_filter1 = avg_csv1(:,1); % total filter time [ms]
avg_build1  = avg_csv1(:,2); % build kd-tree time [ms]
avg_search1 = avg_csv1(:,3); % radius search time [ms]
avg_delete1 = avg_csv1(:,4); % delete time [ms]
avg_knn1    = avg_csv1(:,5); % knn search time [ms]
avg_total1  = avg_csv1(:,6); % total time [ms]
messages1 = csv1(end,1);
avg_operation_percentage1 = runs1 / messages1 * 100; % [%]
printf(" Avg. filter time: %.3f ms \n Avg. build time:  %.3f ms\n Avg. search time: %.3f ms\n Avg. delete time: %.3f ms\n Avg. knn time:    %.3f ms\n Avg. total time:  %.3f ms\n",
        avg_filter1, avg_build1, avg_search1, avg_delete1, avg_knn1, avg_total1);
printf("-- Avg. operation percentage: %.3f\n", avg_operation_percentage1);
disp("");


% file 2
csv2 = csvread("ikd/kd-filter001.csv");
runs2 = rows(csv2) - 2;
avg_csv2 = sum(csv2(2:end-1,:)/1000) / runs2;
avg_filter2 = avg_csv2(:,1); % total filter time [ms]
avg_build2  = avg_csv2(:,2); % build kd-tree time [ms]
avg_search2 = avg_csv2(:,3); % radius search time [ms]
avg_delete2 = avg_csv2(:,4); % delete time [ms]
avg_knn2    = avg_csv2(:,5); % knn search time [ms]
avg_total2  = avg_csv2(:,6); % total time [ms]
messages2 = csv2(end,1);
avg_operation_percentage2 = runs2 / messages2 * 100; % [%]
printf(" Avg. filter time: %.3f ms \n Avg. build time:  %.3f ms\n Avg. search time: %.3f ms\n Avg. delete time: %.3f ms\n Avg. knn time:    %.3f ms\n Avg. total time:  %.3f ms\n",
        avg_filter2, avg_build2, avg_search2, avg_delete2, avg_knn2, avg_total2);
printf("-- Avg. operation percentage: %.3f\n", avg_operation_percentage2);
disp("");

% file 3
csv3 = csvread("ikd/kd-filter002.csv");
runs3 = rows(csv3) - 2;
avg_csv3 = sum(csv3(2:end-1,:)/1000) / runs3;
avg_filter3 = avg_csv3(:,1); % total filter time [ms]
avg_build3  = avg_csv3(:,2); % build kd-tree time [ms]
avg_search3 = avg_csv3(:,3); % radius search time [ms]
avg_delete3 = avg_csv3(:,4); % delete time [ms]
avg_knn3    = avg_csv3(:,5); % knn search time [ms]
avg_total3  = avg_csv3(:,6); % total time [ms]
messages3 = csv3(end,1);
avg_operation_percentage3 = runs3 / messages3 * 100; % [%]
printf(" Avg. filter time: %.3f ms \n Avg. build time:  %.3f ms\n Avg. search time: %.3f ms\n Avg. delete time: %.3f ms\n Avg. knn time:    %.3f ms\n Avg. total time:  %.3f ms\n",
        avg_filter3, avg_build3, avg_search3, avg_delete3, avg_knn3, avg_total3);
printf("-- Avg. operation percentage: %.3f\n", avg_operation_percentage3);
disp("");

% file 4
csv4 = csvread("ikd/kd-filter003.csv");
runs4 = rows(csv4) - 2;
avg_csv4 = sum(csv4(2:end-1,:)/1000) / runs4;
avg_filter4 = avg_csv4(:,1); % total filter time [ms]
avg_build4  = avg_csv4(:,2); % build kd-tree time [ms]
avg_search4 = avg_csv4(:,3); % radius search time [ms]
avg_delete4 = avg_csv4(:,4); % delete time [ms]
avg_knn4    = avg_csv4(:,5); % knn search time [ms]
avg_total4  = avg_csv4(:,6); % total time [ms]
messages4 = csv4(end,1);
avg_operation_percentage4 = runs4 / messages4 * 100; % [%]
printf(" Avg. filter time: %.3f ms \n Avg. build time:  %.3f ms\n Avg. search time: %.3f ms\n Avg. delete time: %.3f ms\n Avg. knn time:    %.3f ms\n Avg. total time:  %.3f ms\n",
        avg_filter4, avg_build4, avg_search4, avg_delete4, avg_knn4, avg_total4);
printf("-- Avg. operation percentage: %.3f\n", avg_operation_percentage4);
disp("");

% file 5
csv5 = csvread("ikd/kd-filter004.csv");
runs5 = rows(csv5) - 2;
avg_csv5 = sum(csv5(2:end-1,:)/1000) / runs5;
avg_filter5 = avg_csv5(:,1); % total filter time [ms]
avg_build5  = avg_csv5(:,2); % build kd-tree time [ms]
avg_search5 = avg_csv5(:,3); % radius search time [ms]
avg_delete5 = avg_csv5(:,4); % delete time [ms]
avg_knn5    = avg_csv5(:,5); % knn search time [ms]
avg_total5  = avg_csv5(:,6); % total time [ms]
messages5 = csv5(end,1);
avg_operation_percentage5 = runs5 / messages5 * 100; % [%]
printf(" Avg. filter time: %.3f ms \n Avg. build time:  %.3f ms\n Avg. search time: %.3f ms\n Avg. delete time: %.3f ms\n Avg. knn time:    %.3f ms\n Avg. total time:  %.3f ms\n",
        avg_filter5, avg_build5, avg_search5, avg_delete5, avg_knn5, avg_total5);
printf("-- Avg. operation percentage: %.3f\n", avg_operation_percentage5);
disp("");


%% combined data
disp("-- TOTAL KD FILTER RESULTS --");
avg_operation_percentage_kd = (avg_operation_percentage1 + avg_operation_percentage2 + avg_operation_percentage3 + avg_operation_percentage4 + avg_operation_percentage5)/5; % [%]
printf(" Avg. operation percentage: %.3f\n\n", avg_operation_percentage_kd);

avg_filter_kd = (avg_filter1 + avg_filter2 + avg_filter3 + avg_filter4 + avg_filter5)/5; % total filter time [ms]
avg_build_kd  = (avg_build1 + avg_build2 + avg_build3 + avg_build4 + avg_build5)/5; % build kd-tree time [ms]
avg_search_kd = (avg_search1 + avg_search2 + avg_search3 + avg_search4 + avg_search5)/5; % radius search time [ms]
avg_delete_kd = (avg_delete1 + avg_delete2 + avg_delete3 + avg_delete4 + avg_delete5)/5; % delete time [ms]
avg_knn_kd    = (avg_knn1 + avg_knn2 + avg_knn3 + avg_knn4 + avg_knn5)/5; % knn search time [ms]
avg_total_kd  = (avg_total1 + avg_total2 + avg_total3 + avg_total4 + avg_total5)/5; % total time [ms]

printf(" Avg. filter time: %.3f ms \n Avg. build time:  %.3f ms\n Avg. search time: %.3f ms\n Avg. delete time: %.3f ms\n Avg. knn time:    %.3f ms\n Avg. total time:  %.3f ms\n",
        avg_filter_kd, avg_build_kd, avg_search_kd, avg_delete_kd, avg_knn_kd, avg_total_kd);



% ----------------------- geo_filter -----------------------
printf("\n\n");
disp("------------ GEO FILTER ------------");

% file 6
csv6 = csvread("ikd/geo-filter000.csv");
runs6 = rows(csv6) - 2;
avg_csv6 = sum(csv6(2:end-1,:)/1000) / runs6;
avg_filter6 = avg_csv6(:,1); % total filter time [ms]
avg_build6  = avg_csv6(:,2); % build kd-tree time [ms]
avg_search6 = avg_csv6(:,3); % radius search time [ms]
avg_delete6 = avg_csv6(:,4); % delete time [ms]
avg_knn6    = avg_csv6(:,5); % knn search time [ms]
avg_total6  = avg_csv6(:,6); % total time [ms]
messages6 = csv6(end,1);
avg_operation_percentage6 = runs6 / messages6 * 100; % [%]
printf(" Avg. filter time: %.3f ms \n Avg. build time:  %.3f ms\n Avg. search time: %.3f ms\n Avg. delete time: %.3f ms\n Avg. knn time:    %.3f ms\n Avg. total time:  %.3f ms\n",
        avg_filter6, avg_build6, avg_search6, avg_delete6, avg_knn6, avg_total6);
printf("-- Avg. operation percentage: %.3f\n", avg_operation_percentage6);
disp("");

% file 7
csv7 = csvread("ikd/geo-filter001.csv");
runs7 = rows(csv7) - 2;
avg_csv7 = sum(csv7(2:end-1,:)/1000) / runs7;
avg_filter7 = avg_csv7(:,1); % total filter time [ms]
avg_build7  = avg_csv7(:,2); % build kd-tree time [ms]
avg_search7 = avg_csv7(:,3); % radius search time [ms]
avg_delete7 = avg_csv7(:,4); % delete time [ms]
avg_knn7    = avg_csv7(:,5); % knn search time [ms]
avg_total7  = avg_csv7(:,6); % total time [ms]
messages7 = csv7(end,1);
avg_operation_percentage7 = runs7 / messages7 * 100; % [%]
printf(" Avg. filter time: %.3f ms \n Avg. build time:  %.3f ms\n Avg. search time: %.3f ms\n Avg. delete time: %.3f ms\n Avg. knn time:    %.3f ms\n Avg. total time:  %.3f ms\n",
        avg_filter7, avg_build7, avg_search7, avg_delete7, avg_knn7, avg_total7);
printf("-- Avg. operation percentage: %.3f\n", avg_operation_percentage7);
disp("");

% file 8
csv8 = csvread("ikd/geo-filter002.csv");
runs8 = rows(csv8) - 2;
avg_csv8 = sum(csv8(2:end-1,:)/1000) / runs8;
avg_filter8 = avg_csv8(:,1); % total filter time [ms]
avg_build8  = avg_csv8(:,2); % build kd-tree time [ms]
avg_search8 = avg_csv8(:,3); % radius search time [ms]
avg_delete8 = avg_csv8(:,4); % delete time [ms]
avg_knn8    = avg_csv8(:,5); % knn search time [ms]
avg_total8  = avg_csv8(:,6); % total time [ms]
messages8 = csv8(end,1);
avg_operation_percentage8 = runs8 / messages8 * 100; % [%]
printf(" Avg. filter time: %.3f ms \n Avg. build time:  %.3f ms\n Avg. search time: %.3f ms\n Avg. delete time: %.3f ms\n Avg. knn time:    %.3f ms\n Avg. total time:  %.3f ms\n",
        avg_filter8, avg_build8, avg_search8, avg_delete8, avg_knn8, avg_total8);
printf("-- Avg. operation percentage: %.3f\n", avg_operation_percentage8);
disp("");

% file 9
csv9 = csvread("ikd/geo-filter003.csv");
runs9 = rows(csv9) - 2;
avg_csv9 = sum(csv9(2:end-1,:)/1000) / runs9;
avg_filter9 = avg_csv9(:,1); % total filter time [ms]
avg_build9  = avg_csv9(:,2); % build kd-tree time [ms]
avg_search9 = avg_csv9(:,3); % radius search time [ms]
avg_delete9 = avg_csv9(:,4); % delete time [ms]
avg_knn9    = avg_csv9(:,5); % knn search time [ms]
avg_total9  = avg_csv9(:,6); % total time [ms]
messages9 = csv9(end,1);
avg_operation_percentage9 = runs9 / messages9 * 100; % [%]
printf(" Avg. filter time: %.3f ms \n Avg. build time:  %.3f ms\n Avg. search time: %.3f ms\n Avg. delete time: %.3f ms\n Avg. knn time:    %.3f ms\n Avg. total time:  %.3f ms\n",
        avg_filter9, avg_build9, avg_search9, avg_delete9, avg_knn9, avg_total9);
printf("-- Avg. operation percentage: %.3f\n", avg_operation_percentage9);
disp("");

% file 10
csv10 = csvread("ikd/geo-filter004.csv");
runs10 = rows(csv10) - 2;
avg_csv10 = sum(csv10(2:end-1,:)/1000) / runs10;
avg_filter10 = avg_csv10(:,1); % total filter time [ms]
avg_build10  = avg_csv10(:,2); % build kd-tree time [ms]
avg_search10 = avg_csv10(:,3); % radius search time [ms]
avg_delete10 = avg_csv10(:,4); % delete time [ms]
avg_knn10    = avg_csv10(:,5); % knn search time [ms]
avg_total10  = avg_csv10(:,6); % total time [ms]
messages10 = csv10(end,1);
avg_operation_percentage10 = runs10 / messages10 * 100; % [%]
printf(" Avg. filter time: %.3f ms \n Avg. build time:  %.3f ms\n Avg. search time: %.3f ms\n Avg. delete time: %.3f ms\n Avg. knn time:    %.3f ms\n Avg. total time:  %.3f ms\n",
        avg_filter10, avg_build10, avg_search10, avg_delete10, avg_knn10, avg_total10);
printf("-- Avg. operation percentage: %.3f\n", avg_operation_percentage10);
disp("");


%% combined data
disp("-- TOTAL GEO FILTER RESULTS --");
avg_operation_percentage_geo = (avg_operation_percentage6 + avg_operation_percentage7 + avg_operation_percentage8 + avg_operation_percentage9 + avg_operation_percentage10)/5; % [%]
printf(" Avg. operation percentage: %.3f\n\n", avg_operation_percentage_geo);

avg_filter_geo = (avg_filter6 + avg_filter7 + avg_filter8 + avg_filter9 + avg_filter10)/5; % total filter time [ms]
avg_build_geo  = (avg_build6 + avg_build7 + avg_build8 + avg_build9 + avg_build10)/5; % build kd-tree time [ms]
avg_search_geo = (avg_search6 + avg_search7 + avg_search8 + avg_search9 + avg_search10)/5; % radius search time [ms]
avg_delete_geo = (avg_delete6 + avg_delete7 + avg_delete8 + avg_delete9 + avg_delete10)/5; % delete time [ms]
avg_knn_geo    = (avg_knn6 + avg_knn7 + avg_knn8 + avg_knn9 + avg_knn10)/5; % knn search time [ms]
avg_total_geo  = (avg_total6 + avg_total7 + avg_total8 + avg_total9 + avg_total10)/5; % total time [ms]

printf(" Avg. filter time: %.3f ms \n Avg. build time:  %.3f ms\n Avg. search time: %.3f ms\n Avg. delete time: %.3f ms\n Avg. knn time:    %.3f ms\n Avg. total time:  %.3f ms\n\n\n\n",
        avg_filter_geo, avg_build_geo, avg_search_geo, avg_delete_geo, avg_knn_geo, avg_total_geo);
