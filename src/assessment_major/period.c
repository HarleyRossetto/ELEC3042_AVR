 #include "period.h"
 
 const uint16_t ADC_TO_PERIOD_TIME_MS_LUT[] = {80, 83, 87, 90, 94, 98, 101, 105, 108, 112, 116, 119, 123, 126, 130, 134, 137, 141, 144, 148, 152, 155, 159, 162, 166, 170, 173, 177, 181, 184, 188, 191, 195, 199, 202, 206, 209, 213, 217, 220, 224, 227, 231, 235, 238, 242, 245, 249, 253, 256, 260, 264, 267, 271, 274, 278, 282, 285, 289, 292, 296, 300, 303, 307, 310, 314, 318, 321, 325, 328, 332, 336, 339, 343, 346, 350, 354, 357, 361, 365, 368, 372, 375, 379, 383, 386, 390, 393, 397, 401, 404, 408, 411, 415, 419, 422, 426, 429, 433, 437, 440, 444, 448, 451, 455, 458, 462, 466, 469, 473, 476, 480, 484, 487, 491, 494, 498, 502, 505, 509, 512, 516, 520, 523, 527, 530, 534, 538, 541, 545, 549, 552, 556, 559, 563, 567, 570, 574, 577, 581, 585, 588, 592, 595, 599, 603, 606, 610, 613, 617, 621, 624, 628, 632, 635, 639, 642, 646, 650, 653, 657, 660, 664, 668, 671, 675, 678, 682, 686, 689, 693, 696, 700, 704, 707, 711, 714, 718, 722, 725, 729, 733, 736, 740, 743, 747, 751, 754, 758, 761, 765, 769, 772, 776, 779, 783, 787, 790, 794, 797, 801, 805, 808, 812, 816, 819, 823, 826, 830, 834, 837, 841, 844, 848, 852, 855, 859, 862, 866, 870, 873, 877, 880, 884, 888, 891, 895, 898, 902, 906, 909, 913, 917, 920, 924, 927, 931, 935, 938, 942, 945, 949, 953, 956, 960, 963, 967, 971, 974, 978, 981, 1000, 1000, 1000, 1000, 1000}; //985, 989, 992, 996, 1000};