clear all;
close all;

## NOTE: In 3DTK y-achso hoch runter d.h. y-3dtk -> z und z-3dtk -> y
##       (nicht -y weil rechtshändiges koordinaten system zu linkshändigem!)
## GT: KNN with k=20 3dtk

% Scenario Physik
#gt_p = [1.8959, 6.8568, 1.8959]; # [deg] Riegl
gt_p = [1.0, 7.0, 1.0, 0.0];    ## NOTE: durch laser distanz messung: höhe = 2.86m länge = 23.02m -> 7.08° steigung auch 7° mit oppo
                           ##       und mit boden 1° steigung (quer zur rampe)
max_entries_ps = 1785;
file_physik_slow = "physik_slow";
max_entries_pf = 1165;
file_physik_fast = "physik_fast";


% Scenario Outside hoch
#gt_dh = [0.0, 11.0, 0.0];        ## NOTE: gemessen mit Oppo! und halt eigentlich keine scharfe kante!
gt_dh = [1.9100, 10.285, 1.8373, 0.0]; #[deg] Riegl
max_entries_dh = 1622;
file_draussen_hoch = "draussen_hoch";


% Scenario Outside runter
#gt_dr = [0.0, 10.0, 20.0, 10.0, 0.0]; ## NOTE: gemessen mit Oppo! und halt eigentlich keine scharfe kante!
gt_dr = [1.8373, 11.898, 19.314, 1.7459];
max_entries_dr = 1380;
file_draussen_runter = "draussen_runter";

% Scenario Outside Tunnel
gt_dt = [0.0, 0.0, 0.0, 0.0];
max_entries_dt = 923;
file_tunnel = "tunnel";

% Scenario Flur
gt_fl = [0.0, 0.0, 0.0, 0.0];
max_entries_fl = 1150;
file_flur = "flur";

% Scenario Wand
gt_w = [0.0, 0.0, 0.0, 0.0];
max_entries_w = 831;
file_wand = "wand";

% Scenario Saal
gt_s = [0.0, 0.0, 0.0, 0.0];
max_entries_s = 738;
file_saal = "saal";


% Evaluate one Scenario
#evalScenario(file_physik_slow, max_entries_ps, gt_p);
#evalScenario(file_physik_fast, max_entries_pf, gt_p);
#evalScenario(file_draussen_hoch, max_entries_dh, gt_dh);
#evalScenario(file_draussen_runter, max_entries_dr, gt_dr);
#evalScenario(file_tunnel, max_entries_dt, gt_dt);
#evalScenario(file_flur, max_entries_fl, gt_fl);
#evalScenario(file_wand, max_entries_w, gt_w);
#evalScenario(file_saal, max_entries_s, gt_s);


% Evaluate RANSAC LSF vs PCA
gt_arr = [gt_fl; gt_s; gt_w; gt_p; gt_p; gt_dh; gt_dr; gt_dt];
max_entries_arr = [max_entries_fl, max_entries_s, max_entries_w, max_entries_ps, max_entries_pf, max_entries_dh, max_entries_dr, max_entries_dt];
evalRansac(max_entries_arr, gt_arr);


