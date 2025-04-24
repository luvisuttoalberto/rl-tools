import re
import matplotlib.pyplot as plt

# Paste the full log data (including all 'Step:' lines) into this string:
raw_data = """
Step: 0/100000 Mean return: 14.0199 Mean episode length: 9999
Step: 1000/100000 Mean return: 5.88984 Mean episode length: 9999
Step: 2000/100000 Mean return: 27.4725 Mean episode length: 9999
Step: 3000/100000 Mean return: 37.4884 Mean episode length: 9999
Step: 4000/100000 Mean return: 8.64391 Mean episode length: 9999
Step: 5000/100000 Mean return: 4.10008 Mean episode length: 9999
Step: 6000/100000 Mean return: 12.4126 Mean episode length: 9999
Step: 7000/100000 Mean return: 44.0348 Mean episode length: 9999
Step: 8000/100000 Mean return: 42.0245 Mean episode length: 9999
Step: 9000/100000 Mean return: 102.407 Mean episode length: 9999
Step: 10000/100000 Mean return: 42.4462 Mean episode length: 9999
Step: 11000/100000 Mean return: 10.5241 Mean episode length: 9999
Step: 12000/100000 Mean return: 96.2112 Mean episode length: 9999
Step: 13000/100000 Mean return: 24.49 Mean episode length: 9999
Step: 14000/100000 Mean return: 42.425 Mean episode length: 9999
Loop step: 14711, env step: 14711, SPS: 1470.83 (elapsed: 10.001 s)
Step: 15000/100000 Mean return: 28.3256 Mean episode length: 9999
Step: 16000/100000 Mean return: 36.4898 Mean episode length: 9999
Step: 17000/100000 Mean return: 48.2416 Mean episode length: 9999
Step: 18000/100000 Mean return: 26.8533 Mean episode length: 9999
Loop step: 18191, env step: 18191, SPS: 347.914 (elapsed: 20.004 s)
Step: 19000/100000 Mean return: 59.2828 Mean episode length: 9999
Step: 20000/100000 Mean return: 23.5067 Mean episode length: 9999
Step: 21000/100000 Mean return: 2.265 Mean episode length: 9999
Loop step: 21051, env step: 21051, SPS: 285.998 (elapsed: 30.004 s)
Step: 22000/100000 Mean return: 10.3464 Mean episode length: 9999
Step: 23000/100000 Mean return: 8.96975 Mean episode length: 9999
Loop step: 23396, env step: 23396, SPS: 234.464 (elapsed: 40.005 s)
Step: 24000/100000 Mean return: 2.514 Mean episode length: 9999
Step: 25000/100000 Mean return: 18.0559 Mean episode length: 9999
Loop step: 25422, env step: 25422, SPS: 202.554 (elapsed: 50.008 s)
Step: 26000/100000 Mean return: 17.3593 Mean episode length: 9999
Step: 27000/100000 Mean return: 2.097 Mean episode length: 9999
Loop step: 27901, env step: 27901, SPS: 247.872 (elapsed: 60.009 s)
Step: 28000/100000 Mean return: 4.45436 Mean episode length: 9999
Step: 29000/100000 Mean return: 2.355 Mean episode length: 9999
Loop step: 29829, env step: 29829, SPS: 192.791 (elapsed: 70.009 s)
Step: 30000/100000 Mean return: 1.197 Mean episode length: 9999
Step: 31000/100000 Mean return: 12.6223 Mean episode length: 9999
Loop step: 31225, env step: 31225, SPS: 139.481 (elapsed: 80.018 s)
Step: 32000/100000 Mean return: 63.5945 Mean episode length: 9999
Loop step: 32925, env step: 32925, SPS: 169.987 (elapsed: 90.019 s)
Step: 33000/100000 Mean return: 23.052 Mean episode length: 9999
Step: 34000/100000 Mean return: 1.506 Mean episode length: 9999
Step: 35000/100000 Mean return: 45.2446 Mean episode length: 9999
Loop step: 35001, env step: 35001, SPS: 205.951 (elapsed: 100.099 s)
Step: 36000/100000 Mean return: 3.48249 Mean episode length: 9999
Loop step: 36796, env step: 36796, SPS: 179.389 (elapsed: 110.105 s)
Step: 37000/100000 Mean return: 23.8523 Mean episode length: 9999
Step: 38000/100000 Mean return: 0.980999 Mean episode length: 9999
Loop step: 38468, env step: 38468, SPS: 167.152 (elapsed: 120.108 s)
Step: 39000/100000 Mean return: 30.5004 Mean episode length: 9999
Step: 40000/100000 Mean return: 73.7339 Mean episode length: 9999
Loop step: 40293, env step: 40293, SPS: 182.433 (elapsed: 130.111 s)
Step: 41000/100000 Mean return: 53.1969 Mean episode length: 9999
Step: 42000/100000 Mean return: 104.028 Mean episode length: 9999
Loop step: 42235, env step: 42235, SPS: 193.997 (elapsed: 140.122 s)
Step: 43000/100000 Mean return: 26.5522 Mean episode length: 9999
Loop step: 43955, env step: 43955, SPS: 171.919 (elapsed: 150.127 s)
Step: 44000/100000 Mean return: 2.052 Mean episode length: 9999
Step: 45000/100000 Mean return: 115.672 Mean episode length: 9999
Loop step: 45681, env step: 45681, SPS: 172.534 (elapsed: 160.13 s)
Step: 46000/100000 Mean return: 29.9252 Mean episode length: 9999
Step: 47000/100000 Mean return: 1.5 Mean episode length: 9999
Loop step: 47843, env step: 47843, SPS: 216.172 (elapsed: 170.132 s)
Step: 48000/100000 Mean return: 54.391 Mean episode length: 9999
Step: 49000/100000 Mean return: 117.02 Mean episode length: 9999
Loop step: 49850, env step: 49850, SPS: 200.657 (elapsed: 180.134 s)
Step: 50000/100000
Step: 50000/100000 Mean return: 182.591 Mean episode length: 9999
Step: 51000/100000 Mean return: 76.3009 Mean episode length: 9999
Loop step: 51745, env step: 51745, SPS: 189.442 (elapsed: 190.137 s)
Step: 52000/100000 Mean return: 4.12858 Mean episode length: 9999
Step: 53000/100000 Mean return: 9.83272 Mean episode length: 9999
Step: 54000/100000 Mean return: 51.0812 Mean episode length: 9999
Loop step: 54001, env step: 54001, SPS: 221.461 (elapsed: 200.324 s)
Step: 55000/100000 Mean return: 73.6183 Mean episode length: 9999
Loop step: 55979, env step: 55979, SPS: 197.737 (elapsed: 210.327 s)
Step: 56000/100000 Mean return: 24.4978 Mean episode length: 9999
Step: 57000/100000 Mean return: 56.0807 Mean episode length: 9999
Step: 58000/100000 Mean return: 15.3409 Mean episode length: 9999
Loop step: 58044, env step: 58044, SPS: 206.426 (elapsed: 220.331 s)
Step: 59000/100000 Mean return: 19.7994 Mean episode length: 9999
Step: 60000/100000 Mean return: 53.1022 Mean episode length: 9999
Loop step: 60138, env step: 60138, SPS: 209.355 (elapsed: 230.333 s)
Step: 61000/100000 Mean return: 64.6995 Mean episode length: 9999
Step: 62000/100000 Mean return: 65.764 Mean episode length: 9999
Loop step: 62091, env step: 62091, SPS: 195.235 (elapsed: 240.336 s)
Step: 63000/100000 Mean return: 94.6656 Mean episode length: 9999
Step: 64000/100000 Mean return: 89.926 Mean episode length: 9999
Loop step: 64033, env step: 64033, SPS: 193.981 (elapsed: 250.347 s)
Step: 65000/100000 Mean return: 113.758 Mean episode length: 9999
Step: 66000/100000 Mean return: 143.929 Mean episode length: 9999
Loop step: 66186, env step: 66186, SPS: 215.294 (elapsed: 260.348 s)
Step: 67000/100000 Mean return: 62.7776 Mean episode length: 9999
Step: 68000/100000 Mean return: 36.4013 Mean episode length: 9999
Loop step: 68273, env step: 68273, SPS: 208.607 (elapsed: 270.352 s)
Step: 69000/100000 Mean return: 88.3936 Mean episode length: 9999
Step: 70000/100000 Mean return: 105.254 Mean episode length: 9999
Loop step: 70055, env step: 70055, SPS: 178.168 (elapsed: 280.354 s)
Step: 71000/100000 Mean return: 204.736 Mean episode length: 9999
Loop step: 71997, env step: 71997, SPS: 193.999 (elapsed: 290.364 s)
Step: 72000/100000 Mean return: 47.7446 Mean episode length: 9999
Step: 73000/100000 Mean return: 10.9462 Mean episode length: 9999
Loop step: 73807, env step: 73807, SPS: 180.989 (elapsed: 300.365 s)
Step: 74000/100000 Mean return: 43.1044 Mean episode length: 9999
Step: 75000/100000 Mean return: 3.186 Mean episode length: 9999
Loop step: 75823, env step: 75823, SPS: 201.548 (elapsed: 310.367 s)
Step: 76000/100000 Mean return: 43.338 Mean episode length: 9999
Step: 77000/100000 Mean return: 65.0766 Mean episode length: 9999
Step: 78000/100000 Mean return: 10.8514 Mean episode length: 9999
Loop step: 78073, env step: 78073, SPS: 224.9 (elapsed: 320.372 s)
Step: 79000/100000 Mean return: 40.8622 Mean episode length: 9999
Step: 80000/100000 Mean return: 1.113 Mean episode length: 9999
Loop step: 80546, env step: 80546, SPS: 247.286 (elapsed: 330.372 s)
Step: 81000/100000 Mean return: 74.1729 Mean episode length: 9999
Step: 82000/100000 Mean return: 41.0939 Mean episode length: 9999
Loop step: 82973, env step: 82973, SPS: 242.654 (elapsed: 340.374 s)
Step: 83000/100000 Mean return: 91.997 Mean episode length: 9999
Step: 84000/100000 Mean return: 65.2967 Mean episode length: 9999
Step: 85000/100000 Mean return: 161.972 Mean episode length: 9999
Loop step: 85281, env step: 85281, SPS: 230.752 (elapsed: 350.376 s)
Step: 86000/100000 Mean return: 70.9206 Mean episode length: 9999
Step: 87000/100000 Mean return: 59.1497 Mean episode length: 9999
Loop step: 87839, env step: 87839, SPS: 255.785 (elapsed: 360.377 s)
Step: 88000/100000 Mean return: 127.706 Mean episode length: 9999
Step: 89000/100000 Mean return: 64.8708 Mean episode length: 9999
Step: 90000/100000 Mean return: 154.113 Mean episode length: 9999
Loop step: 90097, env step: 90097, SPS: 225.78 (elapsed: 370.378 s)
Step: 91000/100000 Mean return: 400.866 Mean episode length: 9999
Step: 92000/100000 Mean return: 292.97 Mean episode length: 9999
Loop step: 92192, env step: 92192, SPS: 209.461 (elapsed: 380.38 s)
Step: 93000/100000 Mean return: 152.475 Mean episode length: 9999
Step: 94000/100000 Mean return: 158.742 Mean episode length: 9999
Step: 95000/100000 Mean return: 131.489 Mean episode length: 9999
Loop step: 95091, env step: 95091, SPS: 289.856 (elapsed: 390.381 s)
Step: 96000/100000 Mean return: 97.7181 Mean episode length: 9999
Step: 97000/100000 Mean return: 72.9173 Mean episode length: 9999
Loop step: 97844, env step: 97844, SPS: 275.25 (elapsed: 400.383 s)
Step: 98000/100000 Mean return: 97.8132 Mean episode length: 9999
Step: 99000/100000 Mean return: 177.804 Mean episode length: 9999
Time: 408.112s
(base) albertoluvisutto@albertoluvisutto-ThinkPad-P1-Gen-3:~/git/rl-tools$ g++ -std=c++17 -Ofast -I include src/rl/environments/multi_agent/oil_platform/sac/cpu/training.cpp -lopenblas -DRL_TOOLS_BACKEND_ENABLE_OPENBLAS
(base) albertoluvisutto@albertoluvisutto-ThinkPad-P1-Gen-3:~/git/rl-tools$ ./a.out 
Step: 0/1000000 Mean return: 14.0199 Mean episode length: 9999
Step: 1000/1000000 Mean return: 5.88984 Mean episode length: 9999
Step: 2000/1000000 Mean return: 27.4725 Mean episode length: 9999
Step: 3000/1000000 Mean return: 37.4884 Mean episode length: 9999
Step: 4000/1000000 Mean return: 8.64391 Mean episode length: 9999
Step: 5000/1000000 Mean return: 4.10008 Mean episode length: 9999
Step: 6000/1000000 Mean return: 12.4126 Mean episode length: 9999
Step: 7000/1000000 Mean return: 44.0348 Mean episode length: 9999
Step: 8000/1000000 Mean return: 42.0245 Mean episode length: 9999
Step: 9000/1000000 Mean return: 102.407 Mean episode length: 9999
Step: 10000/1000000 Mean return: 42.4462 Mean episode length: 9999
Step: 11000/1000000 Mean return: 10.5241 Mean episode length: 9999
Step: 12000/1000000 Mean return: 96.2112 Mean episode length: 9999
Step: 13000/1000000 Mean return: 24.49 Mean episode length: 9999
Step: 14000/1000000 Mean return: 42.425 Mean episode length: 9999
Step: 15000/1000000 Mean return: 28.3256 Mean episode length: 9999
Loop step: 15063, env step: 15063, SPS: 1506.23 (elapsed: 10 s)
Step: 16000/1000000 Mean return: 36.4898 Mean episode length: 9999
Step: 17000/1000000 Mean return: 48.2416 Mean episode length: 9999
Step: 18000/1000000 Mean return: 26.8533 Mean episode length: 9999
Step: 19000/1000000 Mean return: 59.2828 Mean episode length: 9999
Loop step: 19132, env step: 19132, SPS: 406.48 (elapsed: 20.01 s)
Step: 20000/1000000 Mean return: 23.5067 Mean episode length: 9999
Step: 21000/1000000 Mean return: 2.265 Mean episode length: 9999
Loop step: 21591, env step: 21591, SPS: 245.846 (elapsed: 30.012 s)
Step: 22000/1000000 Mean return: 10.3464 Mean episode length: 9999
Step: 23000/1000000 Mean return: 8.96975 Mean episode length: 9999
Step: 24000/1000000 Mean return: 2.514 Mean episode length: 9999
Loop step: 24174, env step: 24174, SPS: 258.296 (elapsed: 40.013 s)
Step: 25000/1000000 Mean return: 18.0559 Mean episode length: 9999
Step: 26000/1000000 Mean return: 17.3593 Mean episode length: 9999
Step: 27000/1000000 Mean return: 2.097 Mean episode length: 9999
Loop step: 27011, env step: 27011, SPS: 283.585 (elapsed: 50.017 s)
Step: 28000/1000000 Mean return: 4.45436 Mean episode length: 9999
Step: 29000/1000000 Mean return: 2.355 Mean episode length: 9999
Loop step: 29849, env step: 29849, SPS: 283.731 (elapsed: 60.019 s)
Step: 30000/1000000 Mean return: 1.197 Mean episode length: 9999
Step: 31000/1000000 Mean return: 12.6223 Mean episode length: 9999
Step: 32000/1000000 Mean return: 63.5945 Mean episode length: 9999
Loop step: 32021, env step: 32021, SPS: 216.993 (elapsed: 70.029 s)
Step: 33000/1000000 Mean return: 23.052 Mean episode length: 9999
Loop step: 33447, env step: 33447, SPS: 142.523 (elapsed: 80.034 s)
Step: 34000/1000000 Mean return: 1.506 Mean episode length: 9999
Loop step: 34521, env step: 34521, SPS: 107.384 (elapsed: 90.036 s)
Step: 35000/1000000 Mean return: 45.2446 Mean episode length: 9999
Loop step: 35690, env step: 35690, SPS: 116.883 (elapsed: 100.037 s)
Step: 36000/1000000 Mean return: 3.48249 Mean episode length: 9999
Loop step: 36820, env step: 36820, SPS: 112.93 (elapsed: 110.043 s)
Step: 37000/1000000 Mean return: 23.8523 Mean episode length: 9999
Step: 38000/1000000 Mean return: 0.980999 Mean episode length: 9999
Loop step: 38109, env step: 38109, SPS: 128.873 (elapsed: 120.045 s)
Step: 39000/1000000 Mean return: 30.5004 Mean episode length: 9999
Loop step: 39090, env step: 39090, SPS: 98.0635 (elapsed: 130.049 s)
Step: 40000/1000000 Mean return: 73.7339 Mean episode length: 9999
Loop step: 40463, env step: 40463, SPS: 137.224 (elapsed: 140.055 s)
Step: 41000/1000000 Mean return: 53.1969 Mean episode length: 9999
Step: 42000/1000000 Mean return: 104.028 Mean episode length: 9999
Step: 43000/1000000 Mean return: 26.5522 Mean episode length: 9999
Loop step: 43153, env step: 43153, SPS: 268.817 (elapsed: 150.061 s)
Step: 44000/1000000 Mean return: 2.052 Mean episode length: 9999
Loop step: 44493, env step: 44493, SPS: 133.963 (elapsed: 160.064 s)
Step: 45000/1000000 Mean return: 115.672 Mean episode length: 9999
Step: 46000/1000000 Mean return: 29.9252 Mean episode length: 9999
Loop step: 46494, env step: 46494, SPS: 199.516 (elapsed: 170.093 s)
Step: 47000/1000000 Mean return: 1.5 Mean episode length: 9999
Step: 48000/1000000 Mean return: 54.391 Mean episode length: 9999
Loop step: 48436, env step: 48436, SPS: 193.991 (elapsed: 180.104 s)
Step: 49000/1000000 Mean return: 117.02 Mean episode length: 9999
Step: 50000/1000000 Mean return: 182.591 Mean episode length: 9999
Loop step: 50676, env step: 50676, SPS: 223.975 (elapsed: 190.105 s)
Step: 51000/1000000 Mean return: 76.3009 Mean episode length: 9999
Step: 52000/1000000 Mean return: 4.12858 Mean episode length: 9999
Step: 53000/1000000 Mean return: 9.83272 Mean episode length: 9999
Loop step: 53466, env step: 53466, SPS: 278.947 (elapsed: 200.107 s)
Step: 54000/1000000 Mean return: 51.0812 Mean episode length: 9999
Step: 55000/1000000 Mean return: 73.6183 Mean episode length: 9999
Step: 56000/1000000 Mean return: 24.4978 Mean episode length: 9999
Loop step: 56341, env step: 56341, SPS: 287.433 (elapsed: 210.109 s)
Step: 57000/1000000 Mean return: 56.0807 Mean episode length: 9999
Step: 58000/1000000 Mean return: 15.3409 Mean episode length: 9999
Step: 59000/1000000 Mean return: 19.7994 Mean episode length: 9999
Loop step: 59084, env step: 59084, SPS: 274.29 (elapsed: 220.11 s)
Step: 60000/1000000 Mean return: 53.1022 Mean episode length: 9999
Step: 61000/1000000 Mean return: 64.6995 Mean episode length: 9999
Step: 62000/1000000 Mean return: 65.764 Mean episode length: 9999
Loop step: 62182, env step: 62182, SPS: 309.758 (elapsed: 230.111 s)
Step: 63000/1000000 Mean return: 94.6656 Mean episode length: 9999
Step: 64000/1000000 Mean return: 89.926 Mean episode length: 9999
Step: 65000/1000000 Mean return: 113.758 Mean episode length: 9999
Loop step: 65283, env step: 65283, SPS: 309.996 (elapsed: 240.115 s)
Step: 66000/1000000 Mean return: 143.929 Mean episode length: 9999
Step: 67000/1000000 Mean return: 62.7776 Mean episode length: 9999
Step: 68000/1000000 Mean return: 36.4013 Mean episode length: 9999
Loop step: 68303, env step: 68303, SPS: 301.861 (elapsed: 250.119 s)
Step: 69000/1000000 Mean return: 88.3936 Mean episode length: 9999
Step: 70000/1000000 Mean return: 105.254 Mean episode length: 9999
Step: 71000/1000000 Mean return: 204.736 Mean episode length: 9999
Loop step: 71174, env step: 71174, SPS: 287.076 (elapsed: 260.12 s)
Step: 72000/1000000 Mean return: 47.7446 Mean episode length: 9999
Step: 73000/1000000 Mean return: 10.9462 Mean episode length: 9999
Step: 74000/1000000 Mean return: 43.1044 Mean episode length: 9999
Loop step: 74234, env step: 74234, SPS: 305.978 (elapsed: 270.121 s)
Step: 75000/1000000 Mean return: 3.186 Mean episode length: 9999
Step: 76000/1000000 Mean return: 43.338 Mean episode length: 9999
Step: 77000/1000000 Mean return: 65.0766 Mean episode length: 9999
Loop step: 77131, env step: 77131, SPS: 289.691 (elapsed: 280.121 s)
Step: 78000/1000000 Mean return: 10.8514 Mean episode length: 9999
Step: 79000/1000000 Mean return: 40.8622 Mean episode length: 9999
Step: 80000/1000000 Mean return: 1.113 Mean episode length: 9999
Loop step: 80135, env step: 80135, SPS: 300.36 (elapsed: 290.122 s)
Step: 81000/1000000 Mean return: 74.1729 Mean episode length: 9999
Step: 82000/1000000 Mean return: 41.0939 Mean episode length: 9999
Step: 83000/1000000 Mean return: 91.997 Mean episode length: 9999
Loop step: 83201, env step: 83201, SPS: 306.515 (elapsed: 300.125 s)
Step: 84000/1000000 Mean return: 65.2967 Mean episode length: 9999
Step: 85000/1000000 Mean return: 161.972 Mean episode length: 9999
Step: 86000/1000000 Mean return: 70.9206 Mean episode length: 9999
Loop step: 86001, env step: 86001, SPS: 279.774 (elapsed: 310.133 s)
Step: 87000/1000000 Mean return: 59.1497 Mean episode length: 9999
Step: 88000/1000000 Mean return: 127.706 Mean episode length: 9999
Loop step: 88793, env step: 88793, SPS: 279.187 (elapsed: 320.134 s)
Step: 89000/1000000 Mean return: 64.8708 Mean episode length: 9999
Step: 90000/1000000 Mean return: 154.113 Mean episode length: 9999
Step: 91000/1000000 Mean return: 400.866 Mean episode length: 9999
Loop step: 91604, env step: 91604, SPS: 281.082 (elapsed: 330.134 s)
Step: 92000/1000000 Mean return: 292.97 Mean episode length: 9999
Step: 93000/1000000 Mean return: 152.475 Mean episode length: 9999
Step: 94000/1000000 Mean return: 158.742 Mean episode length: 9999
Loop step: 94386, env step: 94386, SPS: 278.142 (elapsed: 340.136 s)
Step: 95000/1000000 Mean return: 131.489 Mean episode length: 9999
Step: 96000/1000000 Mean return: 97.7181 Mean episode length: 9999
Loop step: 96971, env step: 96971, SPS: 258.345 (elapsed: 350.142 s)
Step: 97000/1000000 Mean return: 72.9173 Mean episode length: 9999
Step: 98000/1000000 Mean return: 97.8132 Mean episode length: 9999
Step: 99000/1000000 Mean return: 177.804 Mean episode length: 9999
Loop step: 99772, env step: 99772, SPS: 280.098 (elapsed: 360.142 s)
Step: 100000/1000000 Mean return: 178.061 Mean episode length: 9999
Step: 101000/1000000 Mean return: 99.9945 Mean episode length: 9999
Step: 102000/1000000 Mean return: 76.2257 Mean episode length: 9999
Loop step: 102778, env step: 102778, SPS: 300.547 (elapsed: 370.144 s)
Step: 103000/1000000 Mean return: 15.4394 Mean episode length: 9999
Step: 104000/1000000 Mean return: 60.8559 Mean episode length: 9999
Step: 105000/1000000 Mean return: 12.9164 Mean episode length: 9999
Loop step: 105665, env step: 105665, SPS: 288.694 (elapsed: 380.144 s)
Step: 106000/1000000 Mean return: 86.5627 Mean episode length: 9999
Step: 107000/1000000 Mean return: 78.5289 Mean episode length: 9999
Step: 108000/1000000 Mean return: 178.811 Mean episode length: 9999
Loop step: 108565, env step: 108565, SPS: 289.97 (elapsed: 390.145 s)
Step: 109000/1000000 Mean return: 281.384 Mean episode length: 9999
Step: 110000/1000000 Mean return: 268.803 Mean episode length: 9999
Step: 111000/1000000 Mean return: 414.545 Mean episode length: 9999
Loop step: 111525, env step: 111525, SPS: 295.811 (elapsed: 400.152 s)
Step: 112000/1000000 Mean return: 557.33 Mean episode length: 9999
Step: 113000/1000000 Mean return: 270.887 Mean episode length: 9999
Step: 114000/1000000 Mean return: 82.7985 Mean episode length: 9999
Loop step: 114405, env step: 114405, SPS: 287.916 (elapsed: 410.155 s)
Step: 115000/1000000 Mean return: 120.891 Mean episode length: 9999
Step: 116000/1000000 Mean return: 151.849 Mean episode length: 9999
Step: 117000/1000000 Mean return: 460.338 Mean episode length: 9999
Loop step: 117417, env step: 117417, SPS: 301.181 (elapsed: 420.155 s)
Step: 118000/1000000 Mean return: 274.97 Mean episode length: 9999
Step: 119000/1000000 Mean return: 210.903 Mean episode length: 9999
Step: 120000/1000000 Mean return: 248.942 Mean episode length: 9999
Loop step: 120033, env step: 120033, SPS: 261.58 (elapsed: 430.156 s)
Step: 121000/1000000 Mean return: 68.0282 Mean episode length: 9999
Step: 122000/1000000 Mean return: 79.3812 Mean episode length: 9999
Loop step: 122739, env step: 122739, SPS: 270.497 (elapsed: 440.16 s)
Step: 123000/1000000 Mean return: 350.423 Mean episode length: 9999
Step: 124000/1000000 Mean return: 65.1892 Mean episode length: 9999
Step: 125000/1000000 Mean return: 58.3913 Mean episode length: 9999
Loop step: 125503, env step: 125503, SPS: 276.326 (elapsed: 450.163 s)
Step: 126000/1000000 Mean return: 136.968 Mean episode length: 9999
Step: 127000/1000000 Mean return: 279.779 Mean episode length: 9999
Step: 128000/1000000 Mean return: 234.395 Mean episode length: 9999
Loop step: 128149, env step: 128149, SPS: 264.508 (elapsed: 460.166 s)
Step: 129000/1000000 Mean return: 161.277 Mean episode length: 9999
Step: 130000/1000000 Mean return: 455.65 Mean episode length: 9999
Loop step: 130737, env step: 130737, SPS: 258.79 (elapsed: 470.166 s)
Step: 131000/1000000 Mean return: 112.392 Mean episode length: 9999
Step: 132000/1000000 Mean return: 124.326 Mean episode length: 9999
Step: 133000/1000000 Mean return: 285.216 Mean episode length: 9999
Loop step: 133679, env step: 133679, SPS: 294.069 (elapsed: 480.171 s)
Step: 134000/1000000 Mean return: 111.446 Mean episode length: 9999
Step: 135000/1000000 Mean return: 136.455 Mean episode length: 9999
Step: 136000/1000000 Mean return: 116.39 Mean episode length: 9999
Loop step: 136650, env step: 136650, SPS: 296.972 (elapsed: 490.175 s)
Step: 137000/1000000 Mean return: 625.646 Mean episode length: 9999
Step: 138000/1000000 Mean return: 145.661 Mean episode length: 9999
Step: 139000/1000000 Mean return: 134.854 Mean episode length: 9999
Loop step: 139349, env step: 139349, SPS: 269.847 (elapsed: 500.177 s)
Step: 140000/1000000 Mean return: 382.506 Mean episode length: 9999
Step: 141000/1000000 Mean return: 180.215 Mean episode length: 9999
Step: 142000/1000000 Mean return: 59.6488 Mean episode length: 9999
Loop step: 142324, env step: 142324, SPS: 297.485 (elapsed: 510.178 s)
Step: 143000/1000000 Mean return: 123.882 Mean episode length: 9999
Step: 144000/1000000 Mean return: 208.697 Mean episode length: 9999
Step: 145000/1000000 Mean return: 76.9479 Mean episode length: 9999
Loop step: 145272, env step: 145272, SPS: 294.744 (elapsed: 520.18 s)
Step: 146000/1000000 Mean return: 90.6367 Mean episode length: 9999
Step: 147000/1000000 Mean return: 80.3418 Mean episode length: 9999
Step: 148000/1000000 Mean return: 143.264 Mean episode length: 9999
Loop step: 148500, env step: 148500, SPS: 322.792 (elapsed: 530.18 s)
Step: 149000/1000000 Mean return: 76.6366 Mean episode length: 9999
Step: 150000/1000000 Mean return: 9.27923 Mean episode length: 9999
Step: 151000/1000000 Mean return: 16.2422 Mean episode length: 9999
Loop step: 151299, env step: 151299, SPS: 279.715 (elapsed: 540.186 s)
Step: 152000/1000000 Mean return: 2.337 Mean episode length: 9999
Step: 153000/1000000 Mean return: 1.293 Mean episode length: 9999
Step: 154000/1000000 Mean return: 55.2133 Mean episode length: 9999
Loop step: 154257, env step: 154257, SPS: 295.781 (elapsed: 550.187 s)
Step: 155000/1000000 Mean return: 5.28001 Mean episode length: 9999
Step: 156000/1000000 Mean return: 2.085 Mean episode length: 9999
Loop step: 156838, env step: 156838, SPS: 258.066 (elapsed: 560.188 s)
Step: 157000/1000000 Mean return: 4.8 Mean episode length: 9999
Step: 158000/1000000 Mean return: 2.652 Mean episode length: 9999
Step: 159000/1000000 Mean return: 6.22501 Mean episode length: 9999
Loop step: 159376, env step: 159376, SPS: 253.789 (elapsed: 570.189 s)
Step: 160000/1000000 Mean return: 12.2002 Mean episode length: 9999
Step: 161000/1000000 Mean return: 2.46925 Mean episode length: 9999
Loop step: 161965, env step: 161965, SPS: 258.882 (elapsed: 580.19 s)
Step: 162000/1000000 Mean return: 4.87922 Mean episode length: 9999
Step: 163000/1000000 Mean return: 17.6948 Mean episode length: 9999
Step: 164000/1000000 Mean return: 13.545 Mean episode length: 9999
Loop step: 164315, env step: 164315, SPS: 234.923 (elapsed: 590.193 s)
Step: 165000/1000000 Mean return: 50.834 Mean episode length: 9999
Step: 166000/1000000 Mean return: 38.3837 Mean episode length: 9999
Loop step: 166621, env step: 166621, SPS: 230.507 (elapsed: 600.197 s)
Step: 167000/1000000 Mean return: 152.101 Mean episode length: 9999
Step: 168000/1000000 Mean return: 67.4712 Mean episode length: 9999
Step: 169000/1000000 Mean return: 115.676 Mean episode length: 9999
Loop step: 169001, env step: 169001, SPS: 232.728 (elapsed: 610.423 s)
Step: 170000/1000000 Mean return: 93.033 Mean episode length: 9999
Loop step: 170911, env step: 170911, SPS: 190.706 (elapsed: 620.439 s)
Step: 171000/1000000 Mean return: 6.17141 Mean episode length: 9999
Step: 172000/1000000 Mean return: 36.3456 Mean episode length: 9999
Step: 173000/1000000 Mean return: 73.2174 Mean episode length: 9999
Loop step: 173251, env step: 173251, SPS: 233.935 (elapsed: 630.442 s)
Step: 174000/1000000 Mean return: 9.73801 Mean episode length: 9999
Step: 175000/1000000 Mean return: 24.2228 Mean episode length: 9999
Step: 176000/1000000 Mean return: 33.6539 Mean episode length: 9999
Loop step: 176161, env step: 176161, SPS: 290.962 (elapsed: 640.443 s)
Step: 177000/1000000 Mean return: 45.5272 Mean episode length: 9999
Step: 178000/1000000 Mean return: 41.1317 Mean episode length: 9999
Loop step: 178815, env step: 178815, SPS: 265.306 (elapsed: 650.446 s)
Step: 179000/1000000 Mean return: 53.6822 Mean episode length: 9999
Step: 180000/1000000 Mean return: 72.9105 Mean episode length: 9999
Step: 181000/1000000 Mean return: 65.2733 Mean episode length: 9999
Loop step: 181079, env step: 181079, SPS: 226.377 (elapsed: 660.447 s)
Step: 182000/1000000 Mean return: 40.2949 Mean episode length: 9999
Loop step: 182960, env step: 182960, SPS: 187.88 (elapsed: 670.459 s)
Step: 183000/1000000 Mean return: 32.5384 Mean episode length: 9999
Step: 184000/1000000 Mean return: 10.3243 Mean episode length: 9999
Step: 185000/1000000 Mean return: 109.642 Mean episode length: 9999
Loop step: 185027, env step: 185027, SPS: 206.691 (elapsed: 680.46 s)
Step: 186000/1000000 Mean return: 85.9812 Mean episode length: 9999
Step: 187000/1000000 Mean return: 109.778 Mean episode length: 9999
Loop step: 187896, env step: 187896, SPS: 286.847 (elapsed: 690.461 s)
Step: 188000/1000000 Mean return: 14.5419 Mean episode length: 9999
Step: 189000/1000000 Mean return: 28.7975 Mean episode length: 9999
Step: 190000/1000000 Mean return: 6.72978 Mean episode length: 9999
Loop step: 190893, env step: 190893, SPS: 299.627 (elapsed: 700.464 s)
Step: 191000/1000000 Mean return: 122.978 Mean episode length: 9999
Step: 192000/1000000 Mean return: 145.352 Mean episode length: 9999
Step: 193000/1000000 Mean return: 92.7664 Mean episode length: 9999
Loop step: 193939, env step: 193939, SPS: 304.576 (elapsed: 710.465 s)
Step: 194000/1000000 Mean return: 127.376 Mean episode length: 9999
Step: 195000/1000000 Mean return: 95.3368 Mean episode length: 9999
Step: 196000/1000000 Mean return: 103.956 Mean episode length: 9999
Loop step: 196800, env step: 196800, SPS: 286.084 (elapsed: 720.465 s)
Step: 197000/1000000 Mean return: 41.7896 Mean episode length: 9999
Step: 198000/1000000 Mean return: 97.9081 Mean episode length: 9999
Step: 199000/1000000 Mean return: 26.8036 Mean episode length: 9999
Loop step: 199626, env step: 199626, SPS: 282.571 (elapsed: 730.466 s)
Step: 200000/1000000 Mean return: 119.227 Mean episode length: 9999
Step: 201000/1000000 Mean return: 55.9658 Mean episode length: 9999
Loop step: 201809, env step: 201809, SPS: 218.073 (elapsed: 740.477 s)
Step: 202000/1000000 Mean return: 207.915 Mean episode length: 9999
Step: 203000/1000000 Mean return: 115.26 Mean episode length: 9999
Step: 204000/1000000 Mean return: 137.044 Mean episode length: 9999
Loop step: 204072, env step: 204072, SPS: 226.293 (elapsed: 750.477 s)
Step: 205000/1000000 Mean return: 49.9458 Mean episode length: 9999
Step: 206000/1000000 Mean return: 187.109 Mean episode length: 9999
Loop step: 206656, env step: 206656, SPS: 258.352 (elapsed: 760.479 s)
Step: 207000/1000000 Mean return: 60.7049 Mean episode length: 9999
Step: 208000/1000000 Mean return: 117.021 Mean episode length: 9999
Loop step: 208809, env step: 208809, SPS: 215.284 (elapsed: 770.48 s)
Step: 209000/1000000 Mean return: 42.351 Mean episode length: 9999
Step: 210000/1000000 Mean return: 147.25 Mean episode length: 9999
Step: 211000/1000000 Mean return: 201.988 Mean episode length: 9999
Loop step: 211317, env step: 211317, SPS: 250.727 (elapsed: 780.483 s)
Step: 212000/1000000 Mean return: 159.325 Mean episode length: 9999
Step: 213000/1000000 Mean return: 5.78794 Mean episode length: 9999
Step: 214000/1000000 Mean return: 111.667 Mean episode length: 9999
Loop step: 214069, env step: 214069, SPS: 275.139 (elapsed: 790.485 s)
Step: 215000/1000000 Mean return: 181.326 Mean episode length: 9999
Step: 216000/1000000 Mean return: 114.169 Mean episode length: 9999
Loop step: 216639, env step: 216639, SPS: 256.978 (elapsed: 800.486 s)
Step: 217000/1000000 Mean return: 229.364 Mean episode length: 9999
Step: 218000/1000000 Mean return: 173.145 Mean episode length: 9999
Loop step: 218964, env step: 218964, SPS: 232.431 (elapsed: 810.489 s)
Step: 219000/1000000 Mean return: 143.859 Mean episode length: 9999
Step: 220000/1000000 Mean return: 163.822 Mean episode length: 9999
Step: 221000/1000000 Mean return: 234.916 Mean episode length: 9999
Loop step: 221023, env step: 221023, SPS: 205.867 (elapsed: 820.49 s)
Step: 222000/1000000 Mean return: 89.8451 Mean episode length: 9999
Step: 223000/1000000 Mean return: 38.9415 Mean episode length: 9999
Step: 224000/1000000 Mean return: 28.4056 Mean episode length: 9999
Loop step: 224001, env step: 224001, SPS: 295.476 (elapsed: 830.569 s)
Step: 225000/1000000 Mean return: 26.009 Mean episode length: 9999
Step: 226000/1000000 Mean return: 137.033 Mean episode length: 9999
Step: 227000/1000000 Mean return: 143.008 Mean episode length: 9999
Loop step: 227001, env step: 227001, SPS: 293.35 (elapsed: 840.796 s)
Step: 228000/1000000 Mean return: 155.936 Mean episode length: 9999
Step: 229000/1000000 Mean return: 44.0658 Mean episode length: 9999
Loop step: 229363, env step: 229363, SPS: 236.16 (elapsed: 850.797 s)
Step: 230000/1000000 Mean return: 170.91 Mean episode length: 9999
Step: 231000/1000000 Mean return: 102.699 Mean episode length: 9999
Loop step: 231608, env step: 231608, SPS: 224.451 (elapsed: 860.799 s)
Step: 232000/1000000 Mean return: 34.9796 Mean episode length: 9999
Step: 233000/1000000 Mean return: 248.469 Mean episode length: 9999
Loop step: 233992, env step: 233992, SPS: 238.301 (elapsed: 870.804 s)
Step: 234000/1000000 Mean return: 152.511 Mean episode length: 9999
Step: 235000/1000000 Mean return: 117.184 Mean episode length: 9999
Step: 236000/1000000 Mean return: 78.8055 Mean episode length: 9999
Loop step: 236179, env step: 236179, SPS: 218.632 (elapsed: 880.807 s)
Step: 237000/1000000 Mean return: 66.6089 Mean episode length: 9999
Step: 238000/1000000 Mean return: 60.3079 Mean episode length: 9999
Loop step: 238531, env step: 238531, SPS: 235.155 (elapsed: 890.809 s)
Step: 239000/1000000 Mean return: 119.389 Mean episode length: 9999
Step: 240000/1000000 Mean return: 169.699 Mean episode length: 9999
Step: 241000/1000000 Mean return: 218.846 Mean episode length: 9999
Loop step: 241114, env step: 241114, SPS: 258.283 (elapsed: 900.809 s)
Step: 242000/1000000 Mean return: 171.925 Mean episode length: 9999
Step: 243000/1000000 Mean return: 202.232 Mean episode length: 9999
Loop step: 243923, env step: 243923, SPS: 280.826 (elapsed: 910.812 s)
Step: 244000/1000000 Mean return: 114.052 Mean episode length: 9999
Step: 245000/1000000 Mean return: 29.0606 Mean episode length: 9999
Loop step: 245913, env step: 245913, SPS: 198.987 (elapsed: 920.813 s)
Step: 246000/1000000 Mean return: 144.672 Mean episode length: 9999
Step: 247000/1000000 Mean return: 39.6942 Mean episode length: 9999
Step: 248000/1000000 Mean return: 136.812 Mean episode length: 9999
Loop step: 248406, env step: 248406, SPS: 249.26 (elapsed: 930.814 s)
Step: 249000/1000000 Mean return: 20.7204 Mean episode length: 9999
Step: 250000/1000000 Mean return: 47.0942 Mean episode length: 9999
Loop step: 250523, env step: 250523, SPS: 211.635 (elapsed: 940.817 s)
Step: 251000/1000000 Mean return: 161.456 Mean episode length: 9999
Step: 252000/1000000 Mean return: 42.8227 Mean episode length: 9999
Loop step: 252948, env step: 252948, SPS: 242.496 (elapsed: 950.817 s)
Step: 253000/1000000 Mean return: 19.9395 Mean episode length: 9999
Step: 254000/1000000 Mean return: 28.2615 Mean episode length: 9999
Step: 255000/1000000 Mean return: 91.2728 Mean episode length: 9999
Loop step: 255159, env step: 255159, SPS: 221.038 (elapsed: 960.82 s)
Step: 256000/1000000 Mean return: 456.221 Mean episode length: 9999
Step: 257000/1000000 Mean return: 115.456 Mean episode length: 9999
Step: 258000/1000000 Mean return: 33.5906 Mean episode length: 9999
Loop step: 258069, env step: 258069, SPS: 290.994 (elapsed: 970.82 s)
Step: 259000/1000000 Mean return: 70.1968 Mean episode length: 9999
Step: 260000/1000000 Mean return: 193.319 Mean episode length: 9999
Loop step: 260961, env step: 260961, SPS: 289.173 (elapsed: 980.821 s)
Step: 261000/1000000 Mean return: 55.3054 Mean episode length: 9999
Step: 262000/1000000 Mean return: 85.5645 Mean episode length: 9999
Step: 263000/1000000 Mean return: 5.13994 Mean episode length: 9999
Loop step: 263630, env step: 263630, SPS: 266.845 (elapsed: 990.823 s)
Step: 264000/1000000 Mean return: 84.8417 Mean episode length: 9999
Step: 265000/1000000 Mean return: 221.917 Mean episode length: 9999
Step: 266000/1000000 Mean return: 93.2205 Mean episode length: 9999
Loop step: 266439, env step: 266439, SPS: 280.816 (elapsed: 1000.83 s)
Step: 267000/1000000 Mean return: 2.6063 Mean episode length: 9999
Step: 268000/1000000 Mean return: 172.888 Mean episode length: 9999
Step: 269000/1000000 Mean return: 143.862 Mean episode length: 9999
Loop step: 269216, env step: 269216, SPS: 277.682 (elapsed: 1010.83 s)
Step: 270000/1000000 Mean return: 156.403 Mean episode length: 9999
Step: 271000/1000000 Mean return: 250.93 Mean episode length: 9999
Step: 272000/1000000 Mean return: 77.4395 Mean episode length: 9999
Loop step: 272021, env step: 272021, SPS: 280.425 (elapsed: 1020.83 s)
Step: 273000/1000000 Mean return: 208.346 Mean episode length: 9999
Step: 274000/1000000 Mean return: 24.5107 Mean episode length: 9999
Loop step: 274801, env step: 274801, SPS: 277.974 (elapsed: 1030.83 s)
Step: 275000/1000000 Mean return: 58.2157 Mean episode length: 9999
Step: 276000/1000000 Mean return: 211.822 Mean episode length: 9999
Step: 277000/1000000 Mean return: 38.1829 Mean episode length: 9999
Loop step: 277617, env step: 277617, SPS: 281.536 (elapsed: 1040.83 s)
Step: 278000/1000000 Mean return: 44.2977 Mean episode length: 9999
Step: 279000/1000000 Mean return: 72.1644 Mean episode length: 9999
Step: 280000/1000000 Mean return: 183.808 Mean episode length: 9999
Loop step: 280363, env step: 280363, SPS: 274.585 (elapsed: 1050.83 s)
Step: 281000/1000000 Mean return: 35.2349 Mean episode length: 9999
Step: 282000/1000000 Mean return: 131.427 Mean episode length: 9999
Step: 283000/1000000 Mean return: 43.577 Mean episode length: 9999
Loop step: 283333, env step: 283333, SPS: 296.992 (elapsed: 1060.83 s)
Step: 284000/1000000 Mean return: 120.659 Mean episode length: 9999
Step: 285000/1000000 Mean return: 84.7792 Mean episode length: 9999
Step: 286000/1000000 Mean return: 144.393 Mean episode length: 9999
Loop step: 286236, env step: 286236, SPS: 290.261 (elapsed: 1070.84 s)
Step: 287000/1000000 Mean return: 166.932 Mean episode length: 9999
Step: 288000/1000000 Mean return: 95.5266 Mean episode length: 9999
Loop step: 288625, env step: 288625, SPS: 238.542 (elapsed: 1080.85 s)
Step: 289000/1000000 Mean return: 157.032 Mean episode length: 9999
Step: 290000/1000000 Mean return: 11.3869 Mean episode length: 9999
Loop step: 290491, env step: 290491, SPS: 186.587 (elapsed: 1090.85 s)
Step: 291000/1000000 Mean return: 45.4252 Mean episode length: 9999
Step: 292000/1000000 Mean return: 94.9534 Mean episode length: 9999
Step: 293000/1000000 Mean return: 57.7066 Mean episode length: 9999
Loop step: 293001, env step: 293001, SPS: 250.954 (elapsed: 1100.85 s)
Step: 294000/1000000 Mean return: 100.217 Mean episode length: 9999
Step: 295000/1000000 Mean return: 152.041 Mean episode length: 9999
Loop step: 295937, env step: 295937, SPS: 293.458 (elapsed: 1110.86 s)
Step: 296000/1000000 Mean return: 171.915 Mean episode length: 9999
Step: 297000/1000000 Mean return: 272.374 Mean episode length: 9999
Step: 298000/1000000 Mean return: 40.4022 Mean episode length: 9999
Step: 299000/1000000 Mean return: 39.5697 Mean episode length: 9999
Loop step: 299001, env step: 299001, SPS: 302.197 (elapsed: 1121 s)
Step: 300000/1000000 Mean return: 127.729 Mean episode length: 9999
Step: 301000/1000000 Mean return: 184.066 Mean episode length: 9999
Loop step: 301990, env step: 301990, SPS: 298.845 (elapsed: 1131 s)
Step: 302000/1000000 Mean return: 422.883 Mean episode length: 9999
Step: 303000/1000000 Mean return: 444.314 Mean episode length: 9999
Step: 304000/1000000 Mean return: 196.555 Mean episode length: 9999
Loop step: 304937, env step: 304937, SPS: 294.596 (elapsed: 1141 s)
Step: 305000/1000000 Mean return: 216.793 Mean episode length: 9999
Step: 306000/1000000 Mean return: 126.323 Mean episode length: 9999
Step: 307000/1000000 Mean return: 197.405 Mean episode length: 9999
Loop step: 307948, env step: 307948, SPS: 301.091 (elapsed: 1151 s)
Step: 308000/1000000 Mean return: 275.401 Mean episode length: 9999
Step: 309000/1000000 Mean return: 289.754 Mean episode length: 9999
Step: 310000/1000000 Mean return: 190.326 Mean episode length: 9999
Loop step: 310904, env step: 310904, SPS: 295.554 (elapsed: 1161 s)
Step: 311000/1000000 Mean return: 337.268 Mean episode length: 9999
Step: 312000/1000000 Mean return: 205.272 Mean episode length: 9999
Step: 313000/1000000 Mean return: 369.012 Mean episode length: 9999
Loop step: 313909, env step: 313909, SPS: 300.472 (elapsed: 1171.01 s)
Step: 314000/1000000 Mean return: 209.351 Mean episode length: 9999
Step: 315000/1000000 Mean return: 124.798 Mean episode length: 9999
Step: 316000/1000000 Mean return: 140.861 Mean episode length: 9999
Loop step: 316940, env step: 316940, SPS: 303.076 (elapsed: 1181.01 s)
Step: 317000/1000000 Mean return: 4.44 Mean episode length: 9999
Step: 318000/1000000 Mean return: 162.443 Mean episode length: 9999
Step: 319000/1000000 Mean return: 81.8475 Mean episode length: 9999
Loop step: 319773, env step: 319773, SPS: 283.078 (elapsed: 1191.01 s)
Step: 320000/1000000 Mean return: 111.815 Mean episode length: 9999
Step: 321000/1000000 Mean return: 254.009 Mean episode length: 9999
Step: 322000/1000000 Mean return: 93.4794 Mean episode length: 9999
Loop step: 322715, env step: 322715, SPS: 294.149 (elapsed: 1201.02 s)
Step: 323000/1000000 Mean return: 148.428 Mean episode length: 9999
Step: 324000/1000000 Mean return: 16.2177 Mean episode length: 9999
Loop step: 324633, env step: 324633, SPS: 191.776 (elapsed: 1211.02 s)
Step: 325000/1000000 Mean return: 222.106 Mean episode length: 9999
Step: 326000/1000000 Mean return: 299.742 Mean episode length: 9999
Loop step: 326620, env step: 326620, SPS: 198.694 (elapsed: 1221.02 s)
Step: 327000/1000000 Mean return: 82.7116 Mean episode length: 9999
Step: 328000/1000000 Mean return: 21.7482 Mean episode length: 9999
Loop step: 328711, env step: 328711, SPS: 208.751 (elapsed: 1231.03 s)
Step: 329000/1000000 Mean return: 30.6077 Mean episode length: 9999
Step: 330000/1000000 Mean return: 140.982 Mean episode length: 9999
Step: 331000/1000000 Mean return: 50.8689 Mean episode length: 9999
Loop step: 331340, env step: 331340, SPS: 262.889 (elapsed: 1241.03 s)
Step: 332000/1000000 Mean return: 154.763 Mean episode length: 9999
Step: 333000/1000000 Mean return: 132.489 Mean episode length: 9999
Loop step: 333621, env step: 333621, SPS: 228.046 (elapsed: 1251.04 s)
Step: 334000/1000000 Mean return: 474.219 Mean episode length: 9999
Step: 335000/1000000 Mean return: 27.4824 Mean episode length: 9999
Step: 336000/1000000 Mean return: 172.228 Mean episode length: 9999
Loop step: 336271, env step: 336271, SPS: 264.946 (elapsed: 1261.04 s)
Step: 337000/1000000 Mean return: 207.885 Mean episode length: 9999
Step: 338000/1000000 Mean return: 68.1111 Mean episode length: 9999
Loop step: 338594, env step: 338594, SPS: 232.273 (elapsed: 1271.04 s)
Step: 339000/1000000 Mean return: 202.018 Mean episode length: 9999
Step: 340000/1000000 Mean return: 273.395 Mean episode length: 9999
Loop step: 340703, env step: 340703, SPS: 210.833 (elapsed: 1281.04 s)
Step: 341000/1000000 Mean return: 38.1835 Mean episode length: 9999
Step: 342000/1000000 Mean return: 328.967 Mean episode length: 9999
Step: 343000/1000000 Mean return: 177.562 Mean episode length: 9999
Loop step: 343163, env step: 343163, SPS: 245.994 (elapsed: 1291.04 s)
Step: 344000/1000000 Mean return: 172.903 Mean episode length: 9999
Step: 345000/1000000 Mean return: 126.805 Mean episode length: 9999
Loop step: 345604, env step: 345604, SPS: 244.069 (elapsed: 1301.04 s)
Step: 346000/1000000 Mean return: 86.6784 Mean episode length: 9999
Step: 347000/1000000 Mean return: 23.4862 Mean episode length: 9999
Loop step: 347677, env step: 347677, SPS: 207.251 (elapsed: 1311.05 s)
Step: 348000/1000000 Mean return: 114.325 Mean episode length: 9999
Step: 349000/1000000 Mean return: 83.8354 Mean episode length: 9999
Loop step: 349659, env step: 349659, SPS: 198.177 (elapsed: 1321.05 s)
Step: 350000/1000000 Mean return: 183.629 Mean episode length: 9999
Step: 351000/1000000 Mean return: 205.682 Mean episode length: 9999
Loop step: 351659, env step: 351659, SPS: 199.976 (elapsed: 1331.05 s)
Step: 352000/1000000 Mean return: 38.9754 Mean episode length: 9999
Step: 353000/1000000 Mean return: 77.6594 Mean episode length: 9999
Loop step: 353932, env step: 353932, SPS: 227.206 (elapsed: 1341.05 s)
Step: 354000/1000000 Mean return: 179.741 Mean episode length: 9999
Step: 355000/1000000 Mean return: 158.269 Mean episode length: 9999
Step: 356000/1000000 Mean return: 264.095 Mean episode length: 9999
Loop step: 356341, env step: 356341, SPS: 240.791 (elapsed: 1351.06 s)
Step: 357000/1000000 Mean return: 90.3919 Mean episode length: 9999
Step: 358000/1000000 Mean return: 196.852 Mean episode length: 9999
Loop step: 358852, env step: 358852, SPS: 251.092 (elapsed: 1361.06 s)
Step: 359000/1000000 Mean return: 405.462 Mean episode length: 9999
Step: 360000/1000000 Mean return: 172.377 Mean episode length: 9999
Step: 361000/1000000 Mean return: 59.1388 Mean episode length: 9999
Loop step: 361369, env step: 361369, SPS: 251.615 (elapsed: 1371.06 s)
Step: 362000/1000000 Mean return: 72.7433 Mean episode length: 9999
Step: 363000/1000000 Mean return: 56.6259 Mean episode length: 9999
Loop step: 363796, env step: 363796, SPS: 242.664 (elapsed: 1381.06 s)
Step: 364000/1000000 Mean return: 32.7764 Mean episode length: 9999
Step: 365000/1000000 Mean return: 81.4785 Mean episode length: 9999
Step: 366000/1000000 Mean return: 114.469 Mean episode length: 9999
Loop step: 366731, env step: 366731, SPS: 293.483 (elapsed: 1391.06 s)
Step: 367000/1000000 Mean return: 218.227 Mean episode length: 9999
Step: 368000/1000000 Mean return: 28.0872 Mean episode length: 9999
Step: 369000/1000000 Mean return: 344.667 Mean episode length: 9999
Loop step: 369415, env step: 369415, SPS: 268.392 (elapsed: 1401.06 s)
Step: 370000/1000000 Mean return: 119.021 Mean episode length: 9999
Step: 371000/1000000 Mean return: 14.2889 Mean episode length: 9999
Loop step: 371686, env step: 371686, SPS: 227.054 (elapsed: 1411.07 s)
Step: 372000/1000000 Mean return: 185.847 Mean episode length: 9999
Step: 373000/1000000 Mean return: 30.038 Mean episode length: 9999
Step: 374000/1000000 Mean return: 143.215 Mean episode length: 9999
Loop step: 374276, env step: 374276, SPS: 258.958 (elapsed: 1421.07 s)
Step: 375000/1000000 Mean return: 132.314 Mean episode length: 9999
Step: 376000/1000000 Mean return: 244.991 Mean episode length: 9999
Step: 377000/1000000 Mean return: 83.09 Mean episode length: 9999
Loop step: 377277, env step: 377277, SPS: 300.059 (elapsed: 1431.07 s)
Step: 378000/1000000 Mean return: 24.4349 Mean episode length: 9999
Step: 379000/1000000 Mean return: 261.511 Mean episode length: 9999
Step: 380000/1000000 Mean return: 92.7445 Mean episode length: 9999
Loop step: 380359, env step: 380359, SPS: 308.131 (elapsed: 1441.07 s)
Step: 381000/1000000 Mean return: 93.6248 Mean episode length: 9999
Step: 382000/1000000 Mean return: 38.6228 Mean episode length: 9999
Step: 383000/1000000 Mean return: 185.166 Mean episode length: 9999
Loop step: 383603, env step: 383603, SPS: 324.367 (elapsed: 1451.07 s)
Step: 384000/1000000 Mean return: 120.314 Mean episode length: 9999
Step: 385000/1000000 Mean return: 24.7392 Mean episode length: 9999
Step: 386000/1000000 Mean return: 257.515 Mean episode length: 9999
Loop step: 386507, env step: 386507, SPS: 290.375 (elapsed: 1461.07 s)
Step: 387000/1000000 Mean return: 218.002 Mean episode length: 9999
Step: 388000/1000000 Mean return: 140.777 Mean episode length: 9999
Loop step: 388775, env step: 388775, SPS: 226.665 (elapsed: 1471.08 s)
Step: 389000/1000000 Mean return: 89.7488 Mean episode length: 9999
Step: 390000/1000000 Mean return: 182.669 Mean episode length: 9999
Step: 391000/1000000 Mean return: 116.254 Mean episode length: 9999
Loop step: 391772, env step: 391772, SPS: 299.666 (elapsed: 1481.08 s)
Step: 392000/1000000 Mean return: 86.9636 Mean episode length: 9999
Step: 393000/1000000 Mean return: 215.154 Mean episode length: 9999
Step: 394000/1000000 Mean return: 92.8671 Mean episode length: 9999
Loop step: 394701, env step: 394701, SPS: 292.856 (elapsed: 1491.08 s)
Step: 395000/1000000 Mean return: 86.559 Mean episode length: 9999
Step: 396000/1000000 Mean return: 15.7707 Mean episode length: 9999
Step: 397000/1000000 Mean return: 53.3168 Mean episode length: 9999
Loop step: 397631, env step: 397631, SPS: 292.982 (elapsed: 1501.08 s)
Step: 398000/1000000 Mean return: 332.439 Mean episode length: 9999
Step: 399000/1000000 Mean return: 155.997 Mean episode length: 9999
Step: 400000/1000000 Mean return: 18.3444 Mean episode length: 9999
Loop step: 400594, env step: 400594, SPS: 296.295 (elapsed: 1511.08 s)
Step: 401000/1000000 Mean return: 20.835 Mean episode length: 9999
Step: 402000/1000000 Mean return: 105.276 Mean episode length: 9999
Loop step: 402898, env step: 402898, SPS: 230.301 (elapsed: 1521.09 s)
Step: 403000/1000000 Mean return: 78.6004 Mean episode length: 9999
Step: 404000/1000000 Mean return: 164.418 Mean episode length: 9999
Step: 405000/1000000 Mean return: 205.844 Mean episode length: 9999
Loop step: 405463, env step: 405463, SPS: 256.43 (elapsed: 1531.09 s)
Step: 406000/1000000 Mean return: 21.8985 Mean episode length: 9999
Step: 407000/1000000 Mean return: 112.789 Mean episode length: 9999
Step: 408000/1000000 Mean return: 111.218 Mean episode length: 9999
Loop step: 408515, env step: 408515, SPS: 305.19 (elapsed: 1541.09 s)
Step: 409000/1000000 Mean return: 232.072 Mean episode length: 9999
Step: 410000/1000000 Mean return: 216.05 Mean episode length: 9999
Step: 411000/1000000 Mean return: 237.431 Mean episode length: 9999
Loop step: 411455, env step: 411455, SPS: 293.932 (elapsed: 1551.09 s)
Step: 412000/1000000 Mean return: 31.1492 Mean episode length: 9999
Step: 413000/1000000 Mean return: 83.0554 Mean episode length: 9999
Step: 414000/1000000 Mean return: 202.699 Mean episode length: 9999
Loop step: 414537, env step: 414537, SPS: 308.124 (elapsed: 1561.09 s)
Step: 415000/1000000 Mean return: 96.6506 Mean episode length: 9999
Step: 416000/1000000 Mean return: 239.21 Mean episode length: 9999
Step: 417000/1000000 Mean return: 43.5291 Mean episode length: 9999
Loop step: 417362, env step: 417362, SPS: 282.475 (elapsed: 1571.1 s)
Step: 418000/1000000 Mean return: 70.4266 Mean episode length: 9999
Step: 419000/1000000 Mean return: 91.149 Mean episode length: 9999
Step: 420000/1000000 Mean return: 78.0522 Mean episode length: 9999
Loop step: 420297, env step: 420297, SPS: 293.494 (elapsed: 1581.1 s)
Step: 421000/1000000 Mean return: 261.403 Mean episode length: 9999
Step: 422000/1000000 Mean return: 21.7877 Mean episode length: 9999
Step: 423000/1000000 Mean return: 349.015 Mean episode length: 9999
Loop step: 423246, env step: 423246, SPS: 294.892 (elapsed: 1591.1 s)
Step: 424000/1000000 Mean return: 21.7494 Mean episode length: 9999
Step: 425000/1000000 Mean return: 75.8817 Mean episode length: 9999
Step: 426000/1000000 Mean return: 197.216 Mean episode length: 9999
Loop step: 426165, env step: 426165, SPS: 291.834 (elapsed: 1601.1 s)
Step: 427000/1000000 Mean return: 35.5268 Mean episode length: 9999
Step: 428000/1000000 Mean return: 130.756 Mean episode length: 9999
Loop step: 428612, env step: 428612, SPS: 244.684 (elapsed: 1611.1 s)
Step: 429000/1000000 Mean return: 160.085 Mean episode length: 9999
Step: 430000/1000000 Mean return: 134.804 Mean episode length: 9999
Step: 431000/1000000 Mean return: 53.8787 Mean episode length: 9999
Loop step: 431769, env step: 431769, SPS: 315.694 (elapsed: 1621.1 s)
Step: 432000/1000000 Mean return: 44.613 Mean episode length: 9999
Step: 433000/1000000 Mean return: 194.749 Mean episode length: 9999
Step: 434000/1000000 Mean return: 60.8448 Mean episode length: 9999
Loop step: 434755, env step: 434755, SPS: 298.542 (elapsed: 1631.1 s)
Step: 435000/1000000 Mean return: 123.834 Mean episode length: 9999
Step: 436000/1000000 Mean return: 230.607 Mean episode length: 9999
Step: 437000/1000000 Mean return: 175.305 Mean episode length: 9999
Loop step: 437644, env step: 437644, SPS: 288.876 (elapsed: 1641.1 s)
Step: 438000/1000000 Mean return: 102.547 Mean episode length: 9999
Step: 439000/1000000 Mean return: 66.5669 Mean episode length: 9999
Step: 440000/1000000 Mean return: 55.1256 Mean episode length: 9999
Loop step: 440780, env step: 440780, SPS: 313.558 (elapsed: 1651.1 s)
Step: 441000/1000000 Mean return: 159.51 Mean episode length: 9999
Step: 442000/1000000 Mean return: 60.9943 Mean episode length: 9999
Step: 443000/1000000 Mean return: 279.263 Mean episode length: 9999
Loop step: 443843, env step: 443843, SPS: 306.263 (elapsed: 1661.1 s)
Step: 444000/1000000 Mean return: 111.856 Mean episode length: 9999
Step: 445000/1000000 Mean return: 320.654 Mean episode length: 9999
Step: 446000/1000000 Mean return: 187.282 Mean episode length: 9999
Loop step: 446753, env step: 446753, SPS: 290.842 (elapsed: 1671.11 s)
Step: 447000/1000000 Mean return: 160.043 Mean episode length: 9999
Step: 448000/1000000 Mean return: 141.076 Mean episode length: 9999
Step: 449000/1000000 Mean return: 208.276 Mean episode length: 9999
Loop step: 449683, env step: 449683, SPS: 292.796 (elapsed: 1681.12 s)
Step: 450000/1000000 Mean return: 139.297 Mean episode length: 9999
Step: 451000/1000000 Mean return: 162.78 Mean episode length: 9999
Step: 452000/1000000 Mean return: 141.721 Mean episode length: 9999
Loop step: 452758, env step: 452758, SPS: 307.407 (elapsed: 1691.12 s)
Step: 453000/1000000 Mean return: 217.09 Mean episode length: 9999
Step: 454000/1000000 Mean return: 172.992 Mean episode length: 9999
Step: 455000/1000000 Mean return: 114.297 Mean episode length: 9999
Loop step: 455637, env step: 455637, SPS: 287.866 (elapsed: 1701.12 s)
Step: 456000/1000000 Mean return: 129.68 Mean episode length: 9999
Step: 457000/1000000 Mean return: 266.616 Mean episode length: 9999
Step: 458000/1000000 Mean return: 158.454 Mean episode length: 9999
Loop step: 458765, env step: 458765, SPS: 312.739 (elapsed: 1711.12 s)
Step: 459000/1000000 Mean return: 188.341 Mean episode length: 9999
Step: 460000/1000000 Mean return: 253.656 Mean episode length: 9999
Step: 461000/1000000 Mean return: 144.17 Mean episode length: 9999
Loop step: 461863, env step: 461863, SPS: 309.794 (elapsed: 1721.12 s)
Step: 462000/1000000 Mean return: 240.219 Mean episode length: 9999
Step: 463000/1000000 Mean return: 204.797 Mean episode length: 9999
Step: 464000/1000000 Mean return: 67.3439 Mean episode length: 9999
Loop step: 464774, env step: 464774, SPS: 291.066 (elapsed: 1731.12 s)
Step: 465000/1000000 Mean return: 74.6964 Mean episode length: 9999
Step: 466000/1000000 Mean return: 407.262 Mean episode length: 9999
Step: 467000/1000000 Mean return: 89.2323 Mean episode length: 9999
Loop step: 467959, env step: 467959, SPS: 318.469 (elapsed: 1741.12 s)
Step: 468000/1000000 Mean return: 79.076 Mean episode length: 9999
Step: 469000/1000000 Mean return: 230.738 Mean episode length: 9999
Step: 470000/1000000 Mean return: 44.0393 Mean episode length: 9999
Loop step: 470980, env step: 470980, SPS: 302.04 (elapsed: 1751.13 s)
Step: 471000/1000000 Mean return: 31.6269 Mean episode length: 9999
Step: 472000/1000000 Mean return: 197.196 Mean episode length: 9999
Step: 473000/1000000 Mean return: 152.574 Mean episode length: 9999
Step: 474000/1000000 Mean return: 237.578 Mean episode length: 9999
Loop step: 474001, env step: 474001, SPS: 300.301 (elapsed: 1761.19 s)
Step: 475000/1000000 Mean return: 185.657 Mean episode length: 9999
Step: 476000/1000000 Mean return: 166.82 Mean episode length: 9999
Step: 477000/1000000 Mean return: 120.388 Mean episode length: 9999
Loop step: 477167, env step: 477167, SPS: 316.564 (elapsed: 1771.19 s)
Step: 478000/1000000 Mean return: 271.409 Mean episode length: 9999
Step: 479000/1000000 Mean return: 160.263 Mean episode length: 9999
Step: 480000/1000000 Mean return: 318.012 Mean episode length: 9999
Loop step: 480271, env step: 480271, SPS: 310.354 (elapsed: 1781.19 s)
Step: 481000/1000000 Mean return: 180.238 Mean episode length: 9999
Step: 482000/1000000 Mean return: 107.348 Mean episode length: 9999
Step: 483000/1000000 Mean return: 97.2381 Mean episode length: 9999
Loop step: 483188, env step: 483188, SPS: 291.694 (elapsed: 1791.19 s)
Step: 484000/1000000 Mean return: 126.139 Mean episode length: 9999
Step: 485000/1000000 Mean return: 90.3598 Mean episode length: 9999
Step: 486000/1000000 Mean return: 69.8787 Mean episode length: 9999
Loop step: 486451, env step: 486451, SPS: 326.278 (elapsed: 1801.19 s)
Step: 487000/1000000 Mean return: 60.2798 Mean episode length: 9999
Step: 488000/1000000 Mean return: 196.072 Mean episode length: 9999
Step: 489000/1000000 Mean return: 157.43 Mean episode length: 9999
Loop step: 489419, env step: 489419, SPS: 296.798 (elapsed: 1811.19 s)
Step: 490000/1000000 Mean return: 118.225 Mean episode length: 9999
Step: 491000/1000000 Mean return: 170.114 Mean episode length: 9999
Step: 492000/1000000 Mean return: 278.289 Mean episode length: 9999
Loop step: 492291, env step: 492291, SPS: 287.142 (elapsed: 1821.19 s)
Step: 493000/1000000 Mean return: 96.5975 Mean episode length: 9999
Step: 494000/1000000 Mean return: 135.131 Mean episode length: 9999
Step: 495000/1000000 Mean return: 124.822 Mean episode length: 9999
Loop step: 495297, env step: 495297, SPS: 300.516 (elapsed: 1831.19 s)
Step: 496000/1000000 Mean return: 69.3497 Mean episode length: 9999
Step: 497000/1000000 Mean return: 166.524 Mean episode length: 9999
Step: 498000/1000000 Mean return: 155.947 Mean episode length: 9999
Loop step: 498368, env step: 498368, SPS: 307.09 (elapsed: 1841.19 s)
Step: 499000/1000000 Mean return: 185.109 Mean episode length: 9999
Step: 500000/1000000
Step: 500000/1000000 Mean return: 308.118 Mean episode length: 9999
Step: 501000/1000000 Mean return: 239.722 Mean episode length: 9999
Loop step: 501567, env step: 501567, SPS: 319.891 (elapsed: 1851.2 s)
Step: 502000/1000000 Mean return: 200.338 Mean episode length: 9999
Step: 503000/1000000 Mean return: 204.389 Mean episode length: 9999
Step: 504000/1000000 Mean return: 47.8447 Mean episode length: 9999
Loop step: 504649, env step: 504649, SPS: 308.127 (elapsed: 1861.2 s)
Step: 505000/1000000 Mean return: 299.741 Mean episode length: 9999
Step: 506000/1000000 Mean return: 108.416 Mean episode length: 9999
Step: 507000/1000000 Mean return: 131.614 Mean episode length: 9999
Loop step: 507724, env step: 507724, SPS: 307.463 (elapsed: 1871.2 s)
Step: 508000/1000000 Mean return: 224.41 Mean episode length: 9999
Step: 509000/1000000 Mean return: 175.691 Mean episode length: 9999
Step: 510000/1000000 Mean return: 308.817 Mean episode length: 9999
Loop step: 510577, env step: 510577, SPS: 285.243 (elapsed: 1881.2 s)
Step: 511000/1000000 Mean return: 174.815 Mean episode length: 9999
Step: 512000/1000000 Mean return: 208.265 Mean episode length: 9999
Step: 513000/1000000 Mean return: 146.015 Mean episode length: 9999
Loop step: 513493, env step: 513493, SPS: 291.47 (elapsed: 1891.21 s)
Step: 514000/1000000 Mean return: 388.238 Mean episode length: 9999
Step: 515000/1000000 Mean return: 311.157 Mean episode length: 9999
Step: 516000/1000000 Mean return: 236.441 Mean episode length: 9999
Loop step: 516236, env step: 516236, SPS: 274.258 (elapsed: 1901.21 s)
Step: 517000/1000000 Mean return: 139.459 Mean episode length: 9999
Step: 518000/1000000 Mean return: 242.811 Mean episode length: 9999
Loop step: 518964, env step: 518964, SPS: 272.771 (elapsed: 1911.21 s)
Step: 519000/1000000 Mean return: 235.548 Mean episode length: 9999
Step: 520000/1000000 Mean return: 254.792 Mean episode length: 9999
Step: 521000/1000000 Mean return: 159.323 Mean episode length: 9999
Step: 522000/1000000 Mean return: 244.999 Mean episode length: 9999
Loop step: 522001, env step: 522001, SPS: 297.951 (elapsed: 1921.4 s)
Step: 523000/1000000 Mean return: 79.5823 Mean episode length: 9999
Step: 524000/1000000 Mean return: 142.224 Mean episode length: 9999
Step: 525000/1000000 Mean return: 332.495 Mean episode length: 9999
Loop step: 525013, env step: 525013, SPS: 301.196 (elapsed: 1931.4 s)
Step: 526000/1000000 Mean return: 324.411 Mean episode length: 9999
Step: 527000/1000000 Mean return: 492.285 Mean episode length: 9999
Loop step: 527912, env step: 527912, SPS: 289.898 (elapsed: 1941.4 s)
Step: 528000/1000000 Mean return: 233.223 Mean episode length: 9999
Step: 529000/1000000 Mean return: 335.594 Mean episode length: 9999
Step: 530000/1000000 Mean return: 192.789 Mean episode length: 9999
Loop step: 530984, env step: 530984, SPS: 307.171 (elapsed: 1951.4 s)
Step: 531000/1000000 Mean return: 239.984 Mean episode length: 9999
Step: 532000/1000000 Mean return: 225.5 Mean episode length: 9999
Step: 533000/1000000 Mean return: 336.116 Mean episode length: 9999
Loop step: 533893, env step: 533893, SPS: 290.76 (elapsed: 1961.41 s)
Step: 534000/1000000 Mean return: 116.76 Mean episode length: 9999
Step: 535000/1000000 Mean return: 266.76 Mean episode length: 9999
Step: 536000/1000000 Mean return: 176.971 Mean episode length: 9999
Loop step: 536763, env step: 536763, SPS: 286.936 (elapsed: 1971.41 s)
Step: 537000/1000000 Mean return: 451.107 Mean episode length: 9999
Step: 538000/1000000 Mean return: 120.741 Mean episode length: 9999
Step: 539000/1000000 Mean return: 88.1449 Mean episode length: 9999
Loop step: 539628, env step: 539628, SPS: 286.489 (elapsed: 1981.41 s)
Step: 540000/1000000 Mean return: 151.143 Mean episode length: 9999
Step: 541000/1000000 Mean return: 176.56 Mean episode length: 9999
Step: 542000/1000000 Mean return: 159.176 Mean episode length: 9999
Loop step: 542485, env step: 542485, SPS: 285.664 (elapsed: 1991.41 s)
Step: 543000/1000000 Mean return: 348.238 Mean episode length: 9999
Step: 544000/1000000 Mean return: 185.623 Mean episode length: 9999
Step: 545000/1000000 Mean return: 190.282 Mean episode length: 9999
Loop step: 545333, env step: 545333, SPS: 284.799 (elapsed: 2001.41 s)
Step: 546000/1000000 Mean return: 167.926 Mean episode length: 9999
Step: 547000/1000000 Mean return: 283.445 Mean episode length: 9999
Step: 548000/1000000 Mean return: 308.541 Mean episode length: 9999
Loop step: 548394, env step: 548394, SPS: 306.084 (elapsed: 2011.41 s)
Step: 549000/1000000 Mean return: 247.105 Mean episode length: 9999
Step: 550000/1000000 Mean return: 144.061 Mean episode length: 9999
Step: 551000/1000000 Mean return: 222.536 Mean episode length: 9999
Loop step: 551492, env step: 551492, SPS: 309.795 (elapsed: 2021.41 s)
Step: 552000/1000000 Mean return: 233.785 Mean episode length: 9999
Step: 553000/1000000 Mean return: 320.823 Mean episode length: 9999
Step: 554000/1000000 Mean return: 318.535 Mean episode length: 9999
Loop step: 554552, env step: 554552, SPS: 305.933 (elapsed: 2031.41 s)
Step: 555000/1000000 Mean return: 453.13 Mean episode length: 9999
Step: 556000/1000000 Mean return: 341.247 Mean episode length: 9999
Step: 557000/1000000 Mean return: 363.708 Mean episode length: 9999
Loop step: 557632, env step: 557632, SPS: 307.953 (elapsed: 2041.41 s)
Step: 558000/1000000 Mean return: 215.362 Mean episode length: 9999
Step: 559000/1000000 Mean return: 148.556 Mean episode length: 9999
Step: 560000/1000000 Mean return: 351.225 Mean episode length: 9999
Loop step: 560499, env step: 560499, SPS: 286.67 (elapsed: 2051.42 s)
Step: 561000/1000000 Mean return: 185.291 Mean episode length: 9999
Step: 562000/1000000 Mean return: 170.613 Mean episode length: 9999
Step: 563000/1000000 Mean return: 289.539 Mean episode length: 9999
Loop step: 563005, env step: 563005, SPS: 250.504 (elapsed: 2061.42 s)
Step: 564000/1000000 Mean return: 408.493 Mean episode length: 9999
Step: 565000/1000000 Mean return: 233.836 Mean episode length: 9999
Step: 566000/1000000 Mean return: 242.411 Mean episode length: 9999
Loop step: 566054, env step: 566054, SPS: 304.866 (elapsed: 2071.42 s)
Step: 567000/1000000 Mean return: 199.619 Mean episode length: 9999
Step: 568000/1000000 Mean return: 147.893 Mean episode length: 9999
Step: 569000/1000000 Mean return: 426.254 Mean episode length: 9999
Loop step: 569067, env step: 569067, SPS: 301.255 (elapsed: 2081.42 s)
Step: 570000/1000000 Mean return: 166.364 Mean episode length: 9999
Step: 571000/1000000 Mean return: 325.313 Mean episode length: 9999
Loop step: 571991, env step: 571991, SPS: 292.35 (elapsed: 2091.43 s)
Step: 572000/1000000 Mean return: 268.954 Mean episode length: 9999
Step: 573000/1000000 Mean return: 316.881 Mean episode length: 9999
Step: 574000/1000000 Mean return: 366.454 Mean episode length: 9999
Loop step: 574944, env step: 574944, SPS: 295.283 (elapsed: 2101.43 s)
Step: 575000/1000000 Mean return: 315.712 Mean episode length: 9999
Step: 576000/1000000 Mean return: 406.403 Mean episode length: 9999
Step: 577000/1000000 Mean return: 290.091 Mean episode length: 9999
Loop step: 577940, env step: 577940, SPS: 299.57 (elapsed: 2111.43 s)
Step: 578000/1000000 Mean return: 230.054 Mean episode length: 9999
Step: 579000/1000000 Mean return: 261.515 Mean episode length: 9999
Step: 580000/1000000 Mean return: 276.094 Mean episode length: 9999
Loop step: 580789, env step: 580789, SPS: 284.894 (elapsed: 2121.43 s)
Step: 581000/1000000 Mean return: 252.199 Mean episode length: 9999
Step: 582000/1000000 Mean return: 168.898 Mean episode length: 9999
Step: 583000/1000000 Mean return: 150.345 Mean episode length: 9999
Loop step: 583837, env step: 583837, SPS: 304.622 (elapsed: 2131.43 s)
Step: 584000/1000000 Mean return: 425.447 Mean episode length: 9999
Step: 585000/1000000 Mean return: 159.697 Mean episode length: 9999
Step: 586000/1000000 Mean return: 145.475 Mean episode length: 9999
Loop step: 586879, env step: 586879, SPS: 304.103 (elapsed: 2141.43 s)
Step: 587000/1000000 Mean return: 134.456 Mean episode length: 9999
Step: 588000/1000000 Mean return: 271.21 Mean episode length: 9999
Step: 589000/1000000 Mean return: 362.446 Mean episode length: 9999
Step: 590000/1000000 Mean return: 126.691 Mean episode length: 9999
Loop step: 590001, env step: 590001, SPS: 306.859 (elapsed: 2151.61 s)
Step: 591000/1000000 Mean return: 138.155 Mean episode length: 9999
Step: 592000/1000000 Mean return: 137.224 Mean episode length: 9999
Loop step: 592987, env step: 592987, SPS: 298.565 (elapsed: 2161.61 s)
Step: 593000/1000000 Mean return: 218.066 Mean episode length: 9999
Step: 594000/1000000 Mean return: 253.651 Mean episode length: 9999
Step: 595000/1000000 Mean return: 227.886 Mean episode length: 9999
Loop step: 595978, env step: 595978, SPS: 299.026 (elapsed: 2171.61 s)
Step: 596000/1000000 Mean return: 135.72 Mean episode length: 9999
Step: 597000/1000000 Mean return: 154.63 Mean episode length: 9999
Step: 598000/1000000 Mean return: 281.268 Mean episode length: 9999
Loop step: 598725, env step: 598725, SPS: 274.696 (elapsed: 2181.61 s)
Step: 599000/1000000 Mean return: 157.904 Mean episode length: 9999
Step: 600000/1000000 Mean return: 291.012 Mean episode length: 9999
Step: 601000/1000000 Mean return: 257.246 Mean episode length: 9999
Loop step: 601612, env step: 601612, SPS: 288.681 (elapsed: 2191.61 s)
Step: 602000/1000000 Mean return: 228.174 Mean episode length: 9999
Step: 603000/1000000 Mean return: 168.957 Mean episode length: 9999
Step: 604000/1000000 Mean return: 404.864 Mean episode length: 9999
Loop step: 604739, env step: 604739, SPS: 312.611 (elapsed: 2201.62 s)
Step: 605000/1000000 Mean return: 303.064 Mean episode length: 9999
Step: 606000/1000000 Mean return: 317.977 Mean episode length: 9999
Step: 607000/1000000 Mean return: 113.874 Mean episode length: 9999
Loop step: 607879, env step: 607879, SPS: 313.965 (elapsed: 2211.62 s)
Step: 608000/1000000 Mean return: 176.296 Mean episode length: 9999
Step: 609000/1000000 Mean return: 148.064 Mean episode length: 9999
Step: 610000/1000000 Mean return: 401.562 Mean episode length: 9999
Loop step: 610850, env step: 610850, SPS: 297.042 (elapsed: 2221.62 s)
Step: 611000/1000000 Mean return: 189.761 Mean episode length: 9999
Step: 612000/1000000 Mean return: 293.155 Mean episode length: 9999
Step: 613000/1000000 Mean return: 358.018 Mean episode length: 9999
Loop step: 613720, env step: 613720, SPS: 286.974 (elapsed: 2231.62 s)
Step: 614000/1000000 Mean return: 152.007 Mean episode length: 9999
Step: 615000/1000000 Mean return: 222.528 Mean episode length: 9999
Step: 616000/1000000 Mean return: 169.461 Mean episode length: 9999
Loop step: 616783, env step: 616783, SPS: 306.206 (elapsed: 2241.62 s)
Step: 617000/1000000 Mean return: 338.808 Mean episode length: 9999
Step: 618000/1000000 Mean return: 137.328 Mean episode length: 9999
Step: 619000/1000000 Mean return: 139.852 Mean episode length: 9999
Loop step: 619613, env step: 619613, SPS: 282.94 (elapsed: 2251.63 s)
Step: 620000/1000000 Mean return: 155.355 Mean episode length: 9999
Step: 621000/1000000 Mean return: 143.15 Mean episode length: 9999
Step: 622000/1000000 Mean return: 151.263 Mean episode length: 9999
Loop step: 622697, env step: 622697, SPS: 308.327 (elapsed: 2261.63 s)
Step: 623000/1000000 Mean return: 452.155 Mean episode length: 9999
Step: 624000/1000000 Mean return: 105.043 Mean episode length: 9999
Step: 625000/1000000 Mean return: 237.531 Mean episode length: 9999
Loop step: 625703, env step: 625703, SPS: 300.553 (elapsed: 2271.63 s)
Step: 626000/1000000 Mean return: 315.537 Mean episode length: 9999
Step: 627000/1000000 Mean return: 111.733 Mean episode length: 9999
Step: 628000/1000000 Mean return: 98.9756 Mean episode length: 9999
Loop step: 628789, env step: 628789, SPS: 308.513 (elapsed: 2281.63 s)
Step: 629000/1000000 Mean return: 313.248 Mean episode length: 9999
Step: 630000/1000000 Mean return: 218.849 Mean episode length: 9999
Step: 631000/1000000 Mean return: 148.371 Mean episode length: 9999
Loop step: 631750, env step: 631750, SPS: 295.987 (elapsed: 2291.64 s)
Step: 632000/1000000 Mean return: 112.627 Mean episode length: 9999
Step: 633000/1000000 Mean return: 499.699 Mean episode length: 9999
Step: 634000/1000000 Mean return: 223.378 Mean episode length: 9999
Loop step: 634709, env step: 634709, SPS: 295.825 (elapsed: 2301.64 s)
Step: 635000/1000000 Mean return: 186.516 Mean episode length: 9999
Step: 636000/1000000 Mean return: 235.213 Mean episode length: 9999
Step: 637000/1000000 Mean return: 213.675 Mean episode length: 9999
Loop step: 637840, env step: 637840, SPS: 313.077 (elapsed: 2311.64 s)
Step: 638000/1000000 Mean return: 226.008 Mean episode length: 9999
Step: 639000/1000000 Mean return: 168.129 Mean episode length: 9999
Step: 640000/1000000 Mean return: 227.983 Mean episode length: 9999
Loop step: 640867, env step: 640867, SPS: 302.621 (elapsed: 2321.64 s)
Step: 641000/1000000 Mean return: 33.7567 Mean episode length: 9999
Step: 642000/1000000 Mean return: 78.8659 Mean episode length: 9999
Step: 643000/1000000 Mean return: 161.308 Mean episode length: 9999
Loop step: 643878, env step: 643878, SPS: 301.057 (elapsed: 2331.64 s)
Step: 644000/1000000 Mean return: 113.188 Mean episode length: 9999
Step: 645000/1000000 Mean return: 189.098 Mean episode length: 9999
Step: 646000/1000000 Mean return: 187.369 Mean episode length: 9999
Loop step: 646704, env step: 646704, SPS: 282.566 (elapsed: 2341.64 s)
Step: 647000/1000000 Mean return: 157.979 Mean episode length: 9999
Step: 648000/1000000 Mean return: 233.679 Mean episode length: 9999
Step: 649000/1000000 Mean return: 382.891 Mean episode length: 9999
Loop step: 649856, env step: 649856, SPS: 315.143 (elapsed: 2351.65 s)
Step: 650000/1000000 Mean return: 84.348 Mean episode length: 9999
Step: 651000/1000000 Mean return: 241.808 Mean episode length: 9999
Step: 652000/1000000 Mean return: 69.5849 Mean episode length: 9999
Loop step: 652707, env step: 652707, SPS: 285.009 (elapsed: 2361.65 s)
Step: 653000/1000000 Mean return: 123.542 Mean episode length: 9999
Step: 654000/1000000 Mean return: 219.119 Mean episode length: 9999
Step: 655000/1000000 Mean return: 115.32 Mean episode length: 9999
Loop step: 655783, env step: 655783, SPS: 307.59 (elapsed: 2371.65 s)
Step: 656000/1000000 Mean return: 136.097 Mean episode length: 9999
Step: 657000/1000000 Mean return: 102.778 Mean episode length: 9999
Step: 658000/1000000 Mean return: 116.87 Mean episode length: 9999
Loop step: 658792, env step: 658792, SPS: 300.884 (elapsed: 2381.65 s)
Step: 659000/1000000 Mean return: 207.46 Mean episode length: 9999
Step: 660000/1000000 Mean return: 289.42 Mean episode length: 9999
Step: 661000/1000000 Mean return: 103.267 Mean episode length: 9999
Loop step: 661731, env step: 661731, SPS: 293.892 (elapsed: 2391.65 s)
Step: 662000/1000000 Mean return: 184.196 Mean episode length: 9999
Step: 663000/1000000 Mean return: 273.406 Mean episode length: 9999
Step: 664000/1000000 Mean return: 433.186 Mean episode length: 9999
Loop step: 664865, env step: 664865, SPS: 313.341 (elapsed: 2401.65 s)
Step: 665000/1000000 Mean return: 118.506 Mean episode length: 9999
Step: 666000/1000000 Mean return: 161.481 Mean episode length: 9999
Step: 667000/1000000 Mean return: 221.653 Mean episode length: 9999
Loop step: 667915, env step: 667915, SPS: 304.752 (elapsed: 2411.66 s)
Step: 668000/1000000 Mean return: 356.606 Mean episode length: 9999
Step: 669000/1000000 Mean return: 209.402 Mean episode length: 9999
Step: 670000/1000000 Mean return: 163.834 Mean episode length: 9999
Loop step: 670756, env step: 670756, SPS: 284.083 (elapsed: 2421.66 s)
Step: 671000/1000000 Mean return: 252.762 Mean episode length: 9999
Step: 672000/1000000 Mean return: 128.837 Mean episode length: 9999
Step: 673000/1000000 Mean return: 208.771 Mean episode length: 9999
Loop step: 673712, env step: 673712, SPS: 295.595 (elapsed: 2431.66 s)
Step: 674000/1000000 Mean return: 186.208 Mean episode length: 9999
Step: 675000/1000000 Mean return: 196.774 Mean episode length: 9999
Step: 676000/1000000 Mean return: 222.723 Mean episode length: 9999
Loop step: 676623, env step: 676623, SPS: 291.047 (elapsed: 2441.66 s)
Step: 677000/1000000 Mean return: 68.5878 Mean episode length: 9999
Step: 678000/1000000 Mean return: 230.608 Mean episode length: 9999
Step: 679000/1000000 Mean return: 149.318 Mean episode length: 9999
Loop step: 679465, env step: 679465, SPS: 284.192 (elapsed: 2451.66 s)
Step: 680000/1000000 Mean return: 223.441 Mean episode length: 9999
Step: 681000/1000000 Mean return: 328.514 Mean episode length: 9999
Step: 682000/1000000 Mean return: 128.473 Mean episode length: 9999
Loop step: 682412, env step: 682412, SPS: 294.678 (elapsed: 2461.66 s)
Step: 683000/1000000 Mean return: 169.616 Mean episode length: 9999
Step: 684000/1000000 Mean return: 286.902 Mean episode length: 9999
Step: 685000/1000000 Mean return: 153.588 Mean episode length: 9999
Loop step: 685443, env step: 685443, SPS: 303.043 (elapsed: 2471.67 s)
Step: 686000/1000000 Mean return: 86.7391 Mean episode length: 9999
Step: 687000/1000000 Mean return: 176.407 Mean episode length: 9999
Step: 688000/1000000 Mean return: 242.55 Mean episode length: 9999
Loop step: 688469, env step: 688469, SPS: 302.514 (elapsed: 2481.67 s)
Step: 689000/1000000 Mean return: 223.601 Mean episode length: 9999
Step: 690000/1000000 Mean return: 155.999 Mean episode length: 9999
Step: 691000/1000000 Mean return: 150.454 Mean episode length: 9999
Loop step: 691465, env step: 691465, SPS: 299.563 (elapsed: 2491.67 s)
Step: 692000/1000000 Mean return: 216.525 Mean episode length: 9999
Step: 693000/1000000 Mean return: 299.25 Mean episode length: 9999
Step: 694000/1000000 Mean return: 290.525 Mean episode length: 9999
Loop step: 694612, env step: 694612, SPS: 314.644 (elapsed: 2501.67 s)
Step: 695000/1000000 Mean return: 238.621 Mean episode length: 9999
Step: 696000/1000000 Mean return: 122.314 Mean episode length: 9999
Step: 697000/1000000 Mean return: 176.841 Mean episode length: 9999
Loop step: 697819, env step: 697819, SPS: 320.627 (elapsed: 2511.68 s)
Step: 698000/1000000 Mean return: 165.875 Mean episode length: 9999
Step: 699000/1000000 Mean return: 124.457 Mean episode length: 9999
Step: 700000/1000000 Mean return: 229.337 Mean episode length: 9999
Loop step: 700857, env step: 700857, SPS: 303.724 (elapsed: 2521.68 s)
Step: 701000/1000000 Mean return: 153.112 Mean episode length: 9999
Step: 702000/1000000 Mean return: 297.735 Mean episode length: 9999
Step: 703000/1000000 Mean return: 197.894 Mean episode length: 9999
Loop step: 703484, env step: 703484, SPS: 262.7 (elapsed: 2531.68 s)
Step: 704000/1000000 Mean return: 100.158 Mean episode length: 9999
Step: 705000/1000000 Mean return: 172.773 Mean episode length: 9999
Loop step: 705372, env step: 705372, SPS: 188.767 (elapsed: 2541.68 s)
Step: 706000/1000000 Mean return: 135.897 Mean episode length: 9999
Step: 707000/1000000 Mean return: 93.1402 Mean episode length: 9999
Loop step: 707028, env step: 707028, SPS: 165.586 (elapsed: 2551.68 s)
Step: 708000/1000000 Mean return: 140.03 Mean episode length: 9999
Step: 709000/1000000 Mean return: 239.246 Mean episode length: 9999
Loop step: 709293, env step: 709293, SPS: 226.453 (elapsed: 2561.68 s)
Step: 710000/1000000 Mean return: 151.623 Mean episode length: 9999
Step: 711000/1000000 Mean return: 189.38 Mean episode length: 9999
Loop step: 711584, env step: 711584, SPS: 229.065 (elapsed: 2571.68 s)
Step: 712000/1000000 Mean return: 176.542 Mean episode length: 9999
Step: 713000/1000000 Mean return: 249.838 Mean episode length: 9999
Step: 714000/1000000 Mean return: 97.6962 Mean episode length: 9999
Loop step: 714172, env step: 714172, SPS: 258.785 (elapsed: 2581.68 s)
Step: 715000/1000000 Mean return: 628.179 Mean episode length: 9999
Step: 716000/1000000 Mean return: 245.392 Mean episode length: 9999
Loop step: 716713, env step: 716713, SPS: 254.078 (elapsed: 2591.68 s)
Step: 717000/1000000 Mean return: 209.479 Mean episode length: 9999
Step: 718000/1000000 Mean return: 333.843 Mean episode length: 9999
Step: 719000/1000000 Mean return: 143.784 Mean episode length: 9999
Loop step: 719398, env step: 719398, SPS: 268.47 (elapsed: 2601.69 s)
Step: 720000/1000000 Mean return: 228.823 Mean episode length: 9999
Step: 721000/1000000 Mean return: 185.025 Mean episode length: 9999
Loop step: 721834, env step: 721834, SPS: 243.469 (elapsed: 2611.69 s)
Step: 722000/1000000 Mean return: 145.282 Mean episode length: 9999
Step: 723000/1000000 Mean return: 318.411 Mean episode length: 9999
Step: 724000/1000000 Mean return: 138.859 Mean episode length: 9999
Loop step: 724321, env step: 724321, SPS: 248.695 (elapsed: 2621.69 s)
Step: 725000/1000000 Mean return: 170.112 Mean episode length: 9999
Step: 726000/1000000 Mean return: 125.59 Mean episode length: 9999
Loop step: 726517, env step: 726517, SPS: 219.574 (elapsed: 2631.69 s)
Step: 727000/1000000 Mean return: 147.557 Mean episode length: 9999
Step: 728000/1000000 Mean return: 97.0829 Mean episode length: 9999
Loop step: 728296, env step: 728296, SPS: 177.794 (elapsed: 2641.7 s)
Step: 729000/1000000 Mean return: 341.28 Mean episode length: 9999
Step: 730000/1000000 Mean return: 288.409 Mean episode length: 9999
Loop step: 730789, env step: 730789, SPS: 249.27 (elapsed: 2651.7 s)
Step: 731000/1000000 Mean return: 133.561 Mean episode length: 9999
Step: 732000/1000000 Mean return: 264.172 Mean episode length: 9999
Loop step: 732981, env step: 732981, SPS: 219.163 (elapsed: 2661.7 s)
Step: 733000/1000000 Mean return: 173.12 Mean episode length: 9999
Step: 734000/1000000 Mean return: 80.2694 Mean episode length: 9999
Step: 735000/1000000 Mean return: 282.894 Mean episode length: 9999
Loop step: 735144, env step: 735144, SPS: 216.233 (elapsed: 2671.7 s)
Step: 736000/1000000 Mean return: 144.126 Mean episode length: 9999
Step: 737000/1000000 Mean return: 349.502 Mean episode length: 9999
Loop step: 737256, env step: 737256, SPS: 211.163 (elapsed: 2681.71 s)
Step: 738000/1000000 Mean return: 73.9194 Mean episode length: 9999
Step: 739000/1000000 Mean return: 79.1916 Mean episode length: 9999
Loop step: 739086, env step: 739086, SPS: 182.924 (elapsed: 2691.71 s)
Step: 740000/1000000 Mean return: 172.766 Mean episode length: 9999
Step: 741000/1000000 Mean return: 241.532 Mean episode length: 9999
Loop step: 741400, env step: 741400, SPS: 231.372 (elapsed: 2701.71 s)
Step: 742000/1000000 Mean return: 141.987 Mean episode length: 9999
Step: 743000/1000000 Mean return: 68.2228 Mean episode length: 9999
Step: 744000/1000000 Mean return: 34.0943 Mean episode length: 9999
Loop step: 744105, env step: 744105, SPS: 270.475 (elapsed: 2711.71 s)
Step: 745000/1000000 Mean return: 56.2262 Mean episode length: 9999
Step: 746000/1000000 Mean return: 39.3512 Mean episode length: 9999
Loop step: 746510, env step: 746510, SPS: 240.422 (elapsed: 2721.72 s)
Step: 747000/1000000 Mean return: 205.599 Mean episode length: 9999
Step: 748000/1000000 Mean return: 33.0525 Mean episode length: 9999
Step: 749000/1000000 Mean return: 286.708 Mean episode length: 9999
Loop step: 749589, env step: 749589, SPS: 307.825 (elapsed: 2731.72 s)
Step: 750000/1000000 Mean return: 53.5737 Mean episode length: 9999
Step: 751000/1000000 Mean return: 134.056 Mean episode length: 9999
Step: 752000/1000000 Mean return: 210.408 Mean episode length: 9999
Loop step: 752461, env step: 752461, SPS: 287.194 (elapsed: 2741.72 s)
Step: 753000/1000000 Mean return: 208.511 Mean episode length: 9999
Step: 754000/1000000 Mean return: 142.021 Mean episode length: 9999
Step: 755000/1000000 Mean return: 70.8443 Mean episode length: 9999
Loop step: 755400, env step: 755400, SPS: 293.871 (elapsed: 2751.72 s)
Step: 756000/1000000 Mean return: 7.27949 Mean episode length: 9999
Step: 757000/1000000 Mean return: 6.77575 Mean episode length: 9999
Step: 758000/1000000 Mean return: 414.112 Mean episode length: 9999
Loop step: 758531, env step: 758531, SPS: 313.097 (elapsed: 2761.72 s)
Step: 759000/1000000 Mean return: 100.307 Mean episode length: 9999
Step: 760000/1000000 Mean return: 72.7878 Mean episode length: 9999
Step: 761000/1000000 Mean return: 23.0196 Mean episode length: 9999
Loop step: 761626, env step: 761626, SPS: 309.386 (elapsed: 2771.72 s)
Step: 762000/1000000 Mean return: 214.964 Mean episode length: 9999
Step: 763000/1000000 Mean return: 165.871 Mean episode length: 9999
Step: 764000/1000000 Mean return: 31.2947 Mean episode length: 9999
Loop step: 764662, env step: 764662, SPS: 303.581 (elapsed: 2781.72 s)
Step: 765000/1000000 Mean return: 227.892 Mean episode length: 9999
Step: 766000/1000000 Mean return: 373.114 Mean episode length: 9999
Step: 767000/1000000 Mean return: 95.2373 Mean episode length: 9999
Loop step: 767792, env step: 767792, SPS: 312.991 (elapsed: 2791.72 s)
Step: 768000/1000000 Mean return: 125.699 Mean episode length: 9999
Step: 769000/1000000 Mean return: 93.2098 Mean episode length: 9999
Step: 770000/1000000 Mean return: 202.168 Mean episode length: 9999
Loop step: 770804, env step: 770804, SPS: 301.144 (elapsed: 2801.73 s)
Step: 771000/1000000 Mean return: 60.9071 Mean episode length: 9999
Step: 772000/1000000 Mean return: 47.7349 Mean episode length: 9999
Step: 773000/1000000 Mean return: 101.037 Mean episode length: 9999
Loop step: 773903, env step: 773903, SPS: 309.854 (elapsed: 2811.73 s)
Step: 774000/1000000 Mean return: 155.665 Mean episode length: 9999
Step: 775000/1000000 Mean return: 125.811 Mean episode length: 9999
Step: 776000/1000000 Mean return: 201.617 Mean episode length: 9999
Loop step: 776899, env step: 776899, SPS: 299.537 (elapsed: 2821.73 s)
Step: 777000/1000000 Mean return: 44.1271 Mean episode length: 9999
Step: 778000/1000000 Mean return: 12.23 Mean episode length: 9999
Step: 779000/1000000 Mean return: 10.7664 Mean episode length: 9999
Step: 780000/1000000 Mean return: 89.8569 Mean episode length: 9999
Loop step: 780001, env step: 780001, SPS: 303.401 (elapsed: 2831.95 s)
Step: 781000/1000000 Mean return: 55.6565 Mean episode length: 9999
Step: 782000/1000000 Mean return: 66.5054 Mean episode length: 9999
Step: 783000/1000000 Mean return: 200.318 Mean episode length: 9999
Loop step: 783116, env step: 783116, SPS: 311.458 (elapsed: 2841.95 s)
Step: 784000/1000000 Mean return: 149.493 Mean episode length: 9999
Step: 785000/1000000 Mean return: 177.018 Mean episode length: 9999
Step: 786000/1000000 Mean return: 310.692 Mean episode length: 9999
Loop step: 786265, env step: 786265, SPS: 314.778 (elapsed: 2851.96 s)
Step: 787000/1000000 Mean return: 298.674 Mean episode length: 9999
Step: 788000/1000000 Mean return: 70.5378 Mean episode length: 9999
Step: 789000/1000000 Mean return: 50.1039 Mean episode length: 9999
Loop step: 789361, env step: 789361, SPS: 309.442 (elapsed: 2861.96 s)
Step: 790000/1000000 Mean return: 84.4029 Mean episode length: 9999
Step: 791000/1000000 Mean return: 80.5363 Mean episode length: 9999
Step: 792000/1000000 Mean return: 173.426 Mean episode length: 9999
Loop step: 792320, env step: 792320, SPS: 295.872 (elapsed: 2871.97 s)
Step: 793000/1000000 Mean return: 127.499 Mean episode length: 9999
Step: 794000/1000000 Mean return: 271.068 Mean episode length: 9999
Step: 795000/1000000 Mean return: 243.035 Mean episode length: 9999
Loop step: 795265, env step: 795265, SPS: 294.483 (elapsed: 2881.97 s)
Step: 796000/1000000 Mean return: 231.424 Mean episode length: 9999
Step: 797000/1000000 Mean return: 474.289 Mean episode length: 9999
Step: 798000/1000000 Mean return: 225.363 Mean episode length: 9999
Loop step: 798529, env step: 798529, SPS: 326.305 (elapsed: 2891.97 s)
Step: 799000/1000000 Mean return: 102.931 Mean episode length: 9999
Step: 800000/1000000 Mean return: 117.348 Mean episode length: 9999
Step: 801000/1000000 Mean return: 187.222 Mean episode length: 9999
Loop step: 801494, env step: 801494, SPS: 296.373 (elapsed: 2901.97 s)
Step: 802000/1000000 Mean return: 288.56 Mean episode length: 9999
Step: 803000/1000000 Mean return: 86.299 Mean episode length: 9999
Step: 804000/1000000 Mean return: 47.4743 Mean episode length: 9999
Loop step: 804419, env step: 804419, SPS: 292.47 (elapsed: 2911.97 s)
Step: 805000/1000000 Mean return: 221.482 Mean episode length: 9999
Step: 806000/1000000 Mean return: 244.034 Mean episode length: 9999
Step: 807000/1000000 Mean return: 206.169 Mean episode length: 9999
Loop step: 807439, env step: 807439, SPS: 301.963 (elapsed: 2921.97 s)
Step: 808000/1000000 Mean return: 164.205 Mean episode length: 9999
Step: 809000/1000000 Mean return: 87.9857 Mean episode length: 9999
Step: 810000/1000000 Mean return: 310.306 Mean episode length: 9999
Loop step: 810439, env step: 810439, SPS: 299.956 (elapsed: 2931.98 s)
Step: 811000/1000000 Mean return: 265.189 Mean episode length: 9999
Step: 812000/1000000 Mean return: 72.2873 Mean episode length: 9999
Step: 813000/1000000 Mean return: 141.861 Mean episode length: 9999
Loop step: 813493, env step: 813493, SPS: 305.357 (elapsed: 2941.98 s)
Step: 814000/1000000 Mean return: 200.08 Mean episode length: 9999
Step: 815000/1000000 Mean return: 248.591 Mean episode length: 9999
Step: 816000/1000000 Mean return: 224.372 Mean episode length: 9999
Loop step: 816612, env step: 816612, SPS: 311.887 (elapsed: 2951.98 s)
Step: 817000/1000000 Mean return: 202.511 Mean episode length: 9999
Step: 818000/1000000 Mean return: 73.1729 Mean episode length: 9999
Step: 819000/1000000 Mean return: 254.235 Mean episode length: 9999
Loop step: 819666, env step: 819666, SPS: 305.381 (elapsed: 2961.98 s)
Step: 820000/1000000 Mean return: 165.377 Mean episode length: 9999
Step: 821000/1000000 Mean return: 202.005 Mean episode length: 9999
Step: 822000/1000000 Mean return: 274.054 Mean episode length: 9999
Loop step: 822845, env step: 822845, SPS: 317.841 (elapsed: 2971.98 s)
Step: 823000/1000000 Mean return: 210.747 Mean episode length: 9999
Step: 824000/1000000 Mean return: 364.106 Mean episode length: 9999
Step: 825000/1000000 Mean return: 223.853 Mean episode length: 9999
Loop step: 825969, env step: 825969, SPS: 312.366 (elapsed: 2981.98 s)
Step: 826000/1000000 Mean return: 207.054 Mean episode length: 9999
Step: 827000/1000000 Mean return: 90.9485 Mean episode length: 9999
Step: 828000/1000000 Mean return: 291.732 Mean episode length: 9999
Loop step: 828881, env step: 828881, SPS: 291.144 (elapsed: 2991.98 s)
Step: 829000/1000000 Mean return: 162.047 Mean episode length: 9999
Step: 830000/1000000 Mean return: 95.0573 Mean episode length: 9999
Step: 831000/1000000 Mean return: 267.214 Mean episode length: 9999
Loop step: 831888, env step: 831888, SPS: 300.576 (elapsed: 3001.99 s)
Step: 832000/1000000 Mean return: 302.867 Mean episode length: 9999
Step: 833000/1000000 Mean return: 111.69 Mean episode length: 9999
Step: 834000/1000000 Mean return: 228.089 Mean episode length: 9999
Loop step: 834854, env step: 834854, SPS: 296.554 (elapsed: 3011.99 s)
Step: 835000/1000000 Mean return: 233.758 Mean episode length: 9999
Step: 836000/1000000 Mean return: 348.423 Mean episode length: 9999
Step: 837000/1000000 Mean return: 152.615 Mean episode length: 9999
Loop step: 837902, env step: 837902, SPS: 304.757 (elapsed: 3021.99 s)
Step: 838000/1000000 Mean return: 287.614 Mean episode length: 9999
Step: 839000/1000000 Mean return: 307.264 Mean episode length: 9999
Step: 840000/1000000 Mean return: 174.376 Mean episode length: 9999
Loop step: 840963, env step: 840963, SPS: 306.059 (elapsed: 3031.99 s)
Step: 841000/1000000 Mean return: 254.293 Mean episode length: 9999
Step: 842000/1000000 Mean return: 131.894 Mean episode length: 9999
Step: 843000/1000000 Mean return: 139.274 Mean episode length: 9999
Step: 844000/1000000 Mean return: 160.132 Mean episode length: 9999
Loop step: 844001, env step: 844001, SPS: 297.54 (elapsed: 3042.2 s)
Step: 845000/1000000 Mean return: 260.507 Mean episode length: 9999
Step: 846000/1000000 Mean return: 123.932 Mean episode length: 9999
Step: 847000/1000000 Mean return: 188.651 Mean episode length: 9999
Loop step: 847073, env step: 847073, SPS: 307.184 (elapsed: 3052.2 s)
Step: 848000/1000000 Mean return: 225.999 Mean episode length: 9999
Step: 849000/1000000 Mean return: 419.874 Mean episode length: 9999
Step: 850000/1000000 Mean return: 27.2554 Mean episode length: 9999
Loop step: 850071, env step: 850071, SPS: 299.718 (elapsed: 3062.21 s)
Step: 851000/1000000 Mean return: 207.14 Mean episode length: 9999
Step: 852000/1000000 Mean return: 101.124 Mean episode length: 9999
Step: 853000/1000000 Mean return: 156.568 Mean episode length: 9999
Loop step: 853025, env step: 853025, SPS: 295.353 (elapsed: 3072.21 s)
Step: 854000/1000000 Mean return: 196.977 Mean episode length: 9999
Step: 855000/1000000 Mean return: 66.4952 Mean episode length: 9999
Loop step: 855959, env step: 855959, SPS: 293.374 (elapsed: 3082.21 s)
Step: 856000/1000000 Mean return: 176.055 Mean episode length: 9999
Step: 857000/1000000 Mean return: 321.193 Mean episode length: 9999
Step: 858000/1000000 Mean return: 244.576 Mean episode length: 9999
Step: 859000/1000000 Mean return: 197.024 Mean episode length: 9999
Loop step: 859129, env step: 859129, SPS: 316.977 (elapsed: 3092.21 s)
Step: 860000/1000000 Mean return: 245.578 Mean episode length: 9999
Step: 861000/1000000 Mean return: 51.9381 Mean episode length: 9999
Step: 862000/1000000 Mean return: 157.778 Mean episode length: 9999
Loop step: 862118, env step: 862118, SPS: 298.873 (elapsed: 3102.21 s)
Step: 863000/1000000 Mean return: 122.588 Mean episode length: 9999
Step: 864000/1000000 Mean return: 264.001 Mean episode length: 9999
Step: 865000/1000000 Mean return: 192.033 Mean episode length: 9999
Loop step: 865243, env step: 865243, SPS: 312.439 (elapsed: 3112.21 s)
Step: 866000/1000000 Mean return: 113.441 Mean episode length: 9999
Step: 867000/1000000 Mean return: 495.153 Mean episode length: 9999
Step: 868000/1000000 Mean return: 127.114 Mean episode length: 9999
Loop step: 868266, env step: 868266, SPS: 302.255 (elapsed: 3122.21 s)
Step: 869000/1000000 Mean return: 90.4071 Mean episode length: 9999
Step: 870000/1000000 Mean return: 102.926 Mean episode length: 9999
Step: 871000/1000000 Mean return: 137.223 Mean episode length: 9999
Loop step: 871417, env step: 871417, SPS: 315.091 (elapsed: 3132.21 s)
Step: 872000/1000000 Mean return: 235.202 Mean episode length: 9999
Step: 873000/1000000 Mean return: 231.117 Mean episode length: 9999
Step: 874000/1000000 Mean return: 125.479 Mean episode length: 9999
Loop step: 874258, env step: 874258, SPS: 284.055 (elapsed: 3142.22 s)
Step: 875000/1000000 Mean return: 282.441 Mean episode length: 9999
Step: 876000/1000000 Mean return: 270.664 Mean episode length: 9999
Step: 877000/1000000 Mean return: 104.785 Mean episode length: 9999
Loop step: 877395, env step: 877395, SPS: 313.618 (elapsed: 3152.22 s)
Step: 878000/1000000 Mean return: 85.789 Mean episode length: 9999
Step: 879000/1000000 Mean return: 102.726 Mean episode length: 9999
Step: 880000/1000000 Mean return: 171.888 Mean episode length: 9999
Loop step: 880477, env step: 880477, SPS: 308.191 (elapsed: 3162.22 s)
Step: 881000/1000000 Mean return: 183.742 Mean episode length: 9999
Step: 882000/1000000 Mean return: 120.873 Mean episode length: 9999
Step: 883000/1000000 Mean return: 163.205 Mean episode length: 9999
Loop step: 883497, env step: 883497, SPS: 301.815 (elapsed: 3172.22 s)
Step: 884000/1000000 Mean return: 311.273 Mean episode length: 9999
Step: 885000/1000000 Mean return: 340.273 Mean episode length: 9999
Step: 886000/1000000 Mean return: 212.04 Mean episode length: 9999
Loop step: 886548, env step: 886548, SPS: 305.077 (elapsed: 3182.22 s)
Step: 887000/1000000 Mean return: 106.956 Mean episode length: 9999
Step: 888000/1000000 Mean return: 239.566 Mean episode length: 9999
Step: 889000/1000000 Mean return: 247.827 Mean episode length: 9999
Loop step: 889661, env step: 889661, SPS: 311.298 (elapsed: 3192.23 s)
Step: 890000/1000000 Mean return: 359.513 Mean episode length: 9999
Step: 891000/1000000 Mean return: 175.228 Mean episode length: 9999
Step: 892000/1000000 Mean return: 145.994 Mean episode length: 9999
Loop step: 892653, env step: 892653, SPS: 299.195 (elapsed: 3202.23 s)
Step: 893000/1000000 Mean return: 296.001 Mean episode length: 9999
Step: 894000/1000000 Mean return: 66.1147 Mean episode length: 9999
Step: 895000/1000000 Mean return: 102.33 Mean episode length: 9999
Loop step: 895704, env step: 895704, SPS: 304.958 (elapsed: 3212.23 s)
Step: 896000/1000000 Mean return: 265.109 Mean episode length: 9999
Step: 897000/1000000 Mean return: 112.598 Mean episode length: 9999
Step: 898000/1000000 Mean return: 241.239 Mean episode length: 9999
Loop step: 898799, env step: 898799, SPS: 309.438 (elapsed: 3222.23 s)
Step: 899000/1000000 Mean return: 235.645 Mean episode length: 9999
Step: 900000/1000000 Mean return: 139.639 Mean episode length: 9999
Step: 901000/1000000 Mean return: 154.025 Mean episode length: 9999
Loop step: 901716, env step: 901716, SPS: 291.665 (elapsed: 3232.23 s)
Step: 902000/1000000 Mean return: 258.903 Mean episode length: 9999
Step: 903000/1000000 Mean return: 150.8 Mean episode length: 9999
Step: 904000/1000000 Mean return: 324.994 Mean episode length: 9999
Loop step: 904799, env step: 904799, SPS: 308.299 (elapsed: 3242.23 s)
Step: 905000/1000000 Mean return: 367.583 Mean episode length: 9999
Step: 906000/1000000 Mean return: 71.5586 Mean episode length: 9999
Step: 907000/1000000 Mean return: 217.92 Mean episode length: 9999
Step: 908000/1000000 Mean return: 182.858 Mean episode length: 9999
Loop step: 908001, env step: 908001, SPS: 318.733 (elapsed: 3252.28 s)
Step: 909000/1000000 Mean return: 124.195 Mean episode length: 9999
Step: 910000/1000000 Mean return: 444.184 Mean episode length: 9999
Step: 911000/1000000 Mean return: 142.462 Mean episode length: 9999
Loop step: 911001, env step: 911001, SPS: 298.079 (elapsed: 3262.34 s)
Step: 912000/1000000 Mean return: 160.662 Mean episode length: 9999
Step: 913000/1000000 Mean return: 295.863 Mean episode length: 9999
Step: 914000/1000000 Mean return: 151.188 Mean episode length: 9999
Loop step: 914090, env step: 914090, SPS: 308.869 (elapsed: 3272.35 s)
Step: 915000/1000000 Mean return: 309.271 Mean episode length: 9999
Step: 916000/1000000 Mean return: 178.149 Mean episode length: 9999
Step: 917000/1000000 Mean return: 403.29 Mean episode length: 9999
Loop step: 917095, env step: 917095, SPS: 300.463 (elapsed: 3282.35 s)
Step: 918000/1000000 Mean return: 212.394 Mean episode length: 9999
Step: 919000/1000000 Mean return: 202.827 Mean episode length: 9999
Step: 920000/1000000 Mean return: 39.7338 Mean episode length: 9999
Loop step: 920215, env step: 920215, SPS: 311.939 (elapsed: 3292.35 s)
Step: 921000/1000000 Mean return: 59.9104 Mean episode length: 9999
Step: 922000/1000000 Mean return: 90.6704 Mean episode length: 9999
Step: 923000/1000000 Mean return: 58.6005 Mean episode length: 9999
Loop step: 923001, env step: 923001, SPS: 277.392 (elapsed: 3302.39 s)
Step: 924000/1000000 Mean return: 241.664 Mean episode length: 9999
Step: 925000/1000000 Mean return: 233.117 Mean episode length: 9999
Loop step: 925934, env step: 925934, SPS: 293.296 (elapsed: 3312.39 s)
Step: 926000/1000000 Mean return: 84.9808 Mean episode length: 9999
Step: 927000/1000000 Mean return: 120.948 Mean episode length: 9999
Step: 928000/1000000 Mean return: 86.8475 Mean episode length: 9999
Loop step: 928834, env step: 928834, SPS: 289.996 (elapsed: 3322.39 s)
Step: 929000/1000000 Mean return: 157.252 Mean episode length: 9999
Step: 930000/1000000 Mean return: 98.8905 Mean episode length: 9999
Step: 931000/1000000 Mean return: 107.869 Mean episode length: 9999
Loop step: 931732, env step: 931732, SPS: 289.724 (elapsed: 3332.39 s)
Step: 932000/1000000 Mean return: 113.523 Mean episode length: 9999
Step: 933000/1000000 Mean return: 139.067 Mean episode length: 9999
Step: 934000/1000000 Mean return: 134.794 Mean episode length: 9999
Loop step: 934731, env step: 934731, SPS: 299.858 (elapsed: 3342.4 s)
Step: 935000/1000000 Mean return: 207.953 Mean episode length: 9999
Step: 936000/1000000 Mean return: 260.804 Mean episode length: 9999
Step: 937000/1000000 Mean return: 133.113 Mean episode length: 9999
Loop step: 937917, env step: 937917, SPS: 318.527 (elapsed: 3352.4 s)
Step: 938000/1000000 Mean return: 242.852 Mean episode length: 9999
Step: 939000/1000000 Mean return: 107.406 Mean episode length: 9999
Step: 940000/1000000 Mean return: 104.088 Mean episode length: 9999
Step: 941000/1000000 Mean return: 276.166 Mean episode length: 9999
Loop step: 941001, env step: 941001, SPS: 304.199 (elapsed: 3362.54 s)
Step: 942000/1000000 Mean return: 273.551 Mean episode length: 9999
Step: 943000/1000000 Mean return: 150.165 Mean episode length: 9999
Step: 944000/1000000 Mean return: 28.0675 Mean episode length: 9999
Loop step: 944326, env step: 944326, SPS: 332.494 (elapsed: 3372.54 s)
Step: 945000/1000000 Mean return: 194.733 Mean episode length: 9999
Step: 946000/1000000 Mean return: 110.149 Mean episode length: 9999
Step: 947000/1000000 Mean return: 87.4146 Mean episode length: 9999
Loop step: 947234, env step: 947234, SPS: 290.683 (elapsed: 3382.54 s)
Step: 948000/1000000 Mean return: 323.302 Mean episode length: 9999
Step: 949000/1000000 Mean return: 215.255 Mean episode length: 9999
Step: 950000/1000000 Mean return: 112.355 Mean episode length: 9999
Loop step: 950186, env step: 950186, SPS: 295.196 (elapsed: 3392.54 s)
Step: 951000/1000000 Mean return: 65.5525 Mean episode length: 9999
Step: 952000/1000000 Mean return: 138.655 Mean episode length: 9999
Step: 953000/1000000 Mean return: 219.887 Mean episode length: 9999
Loop step: 953178, env step: 953178, SPS: 299.128 (elapsed: 3402.54 s)
Step: 954000/1000000 Mean return: 76.3635 Mean episode length: 9999
Step: 955000/1000000 Mean return: 115.042 Mean episode length: 9999
Step: 956000/1000000 Mean return: 194.383 Mean episode length: 9999
Loop step: 956121, env step: 956121, SPS: 294.25 (elapsed: 3412.55 s)
Step: 957000/1000000 Mean return: 224.169 Mean episode length: 9999
Step: 958000/1000000 Mean return: 248.051 Mean episode length: 9999
Step: 959000/1000000 Mean return: 107.569 Mean episode length: 9999
Loop step: 959047, env step: 959047, SPS: 292.578 (elapsed: 3422.55 s)
Step: 960000/1000000 Mean return: 225.105 Mean episode length: 9999
Step: 961000/1000000 Mean return: 125.548 Mean episode length: 9999
Step: 962000/1000000 Mean return: 101.713 Mean episode length: 9999
Loop step: 962026, env step: 962026, SPS: 297.898 (elapsed: 3432.55 s)
Step: 963000/1000000 Mean return: 624.467 Mean episode length: 9999
Step: 964000/1000000 Mean return: 177.517 Mean episode length: 9999
Loop step: 964952, env step: 964952, SPS: 292.576 (elapsed: 3442.55 s)
Step: 965000/1000000 Mean return: 143.385 Mean episode length: 9999
Step: 966000/1000000 Mean return: 175.843 Mean episode length: 9999
Step: 967000/1000000 Mean return: 163.854 Mean episode length: 9999
Step: 968000/1000000 Mean return: 244.561 Mean episode length: 9999
Loop step: 968069, env step: 968069, SPS: 311.675 (elapsed: 3452.55 s)
Step: 969000/1000000 Mean return: 116.156 Mean episode length: 9999
Step: 970000/1000000 Mean return: 97.7668 Mean episode length: 9999
Step: 971000/1000000 Mean return: 229.718 Mean episode length: 9999
Loop step: 971213, env step: 971213, SPS: 314.32 (elapsed: 3462.55 s)
Step: 972000/1000000 Mean return: 67.4766 Mean episode length: 9999
Step: 973000/1000000 Mean return: 119.181 Mean episode length: 9999
Step: 974000/1000000 Mean return: 72.4382 Mean episode length: 9999
Loop step: 974283, env step: 974283, SPS: 306.991 (elapsed: 3472.55 s)
Step: 975000/1000000 Mean return: 165.535 Mean episode length: 9999
Step: 976000/1000000 Mean return: 38.6044 Mean episode length: 9999
Step: 977000/1000000 Mean return: 105.296 Mean episode length: 9999
Loop step: 977282, env step: 977282, SPS: 299.872 (elapsed: 3482.55 s)
Step: 978000/1000000 Mean return: 61.1928 Mean episode length: 9999
Step: 979000/1000000 Mean return: 28.763 Mean episode length: 9999
Step: 980000/1000000 Mean return: 76.9879 Mean episode length: 9999
Loop step: 980454, env step: 980454, SPS: 317.036 (elapsed: 3492.56 s)
Step: 981000/1000000 Mean return: 67.5473 Mean episode length: 9999
Step: 982000/1000000 Mean return: 83.6769 Mean episode length: 9999
Step: 983000/1000000 Mean return: 146.338 Mean episode length: 9999
Loop step: 983279, env step: 983279, SPS: 282.412 (elapsed: 3502.56 s)
Step: 984000/1000000 Mean return: 58.5974 Mean episode length: 9999
Step: 985000/1000000 Mean return: 203.172 Mean episode length: 9999
Step: 986000/1000000 Mean return: 283.853 Mean episode length: 9999
Loop step: 986397, env step: 986397, SPS: 311.773 (elapsed: 3512.56 s)
Step: 987000/1000000 Mean return: 292.912 Mean episode length: 9999
Step: 988000/1000000 Mean return: 42.6115 Mean episode length: 9999
Step: 989000/1000000 Mean return: 233.508 Mean episode length: 9999
Loop step: 989455, env step: 989455, SPS: 305.659 (elapsed: 3522.57 s)
Step: 990000/1000000 Mean return: 107.761 Mean episode length: 9999
Step: 991000/1000000 Mean return: 319.276 Mean episode length: 9999
Step: 992000/1000000 Mean return: 264.836 Mean episode length: 9999
Loop step: 992316, env step: 992316, SPS: 286.056 (elapsed: 3532.57 s)
Step: 993000/1000000 Mean return: 57.7228 Mean episode length: 9999
Step: 994000/1000000 Mean return: 115.521 Mean episode length: 9999
Step: 995000/1000000 Mean return: 73.5098 Mean episode length: 9999
Loop step: 995321, env step: 995321, SPS: 300.46 (elapsed: 3542.57 s)
Step: 996000/1000000 Mean return: 162.812 Mean episode length: 9999
Step: 997000/1000000 Mean return: 55.0858 Mean episode length: 9999
Step: 998000/1000000 Mean return: 168.202 Mean episode length: 9999
Loop step: 998254, env step: 998254, SPS: 292.633 (elapsed: 3552.59 s)
Step: 999000/1000000 Mean return: 114.615 Mean episode length: 9999
Time: 3558.49s
"""

# Extract steps and returns
steps = []
returns = []
for line in raw_data.splitlines():
    m = re.match(r"Step:\s*(\d+)/\d+\s+Mean return:\s*([\d\.]+)", line)
    if m:
        steps.append(int(m.group(1)))
        returns.append(float(m.group(2)))

# Plot
plt.figure()
plt.plot(steps, returns)
plt.xlabel("Training Step")
plt.ylabel("Mean Return")
plt.title("Mean Return vs Training Step (0 to 999000)")
plt.show()
