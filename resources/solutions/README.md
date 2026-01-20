## Solutions

### Studies

We provide detailed solutions for computational studies. 

`case_study/` contains all solutions regarding the main case study of NYC

`computational_study/` contains all solution concerning the comparison of the three proposed approach: Sequential Matching and Dispatching (MATH); Decomposition-based metaheuristic approach (ILS); ILS warm-started with the solutions from MATH

`pdptw/` contains all solutions found during our final run of experiments for the reference benchmark of [Sartori&Buriol](https://github.com/cssartori/pdptw-instances).

### Tuning

`MATH_tuning/` contains the solutions of the tuning process for the MATH approach.

`ILS_tuning/` contains the solutions of the tuning process for the ILS approach.

----------

Note: The solution files are compressed using [7-zip](https://7-zip.org/), located in their respective folders.

----------

### Solution file structure

Solutions files are structured similar to the PDPTW and VRPTW solutions maintained by [SINTEF](https://www.sintef.no/projectweb/top/pdptw/) with added solver information. A sample of a file can be found below:

```
Instance name:    <name of the instance>
Authors:          <authors>
Date:             <a reference date>
Reference:        <the title of a reference that generated the solution>
Solution: <additional solver information: #unassigned customers;#vehicles used;objective;resolution time in seconds>
Route 1 :  348 349 1712 1794 1713 1795 1888 2986 3558 2987 1889 3559 5786 5787 6872 6873 7614 7615 8122 8500 8123 8501
Route 2 :  226 227 1596 2012 1597 2712 2013 2713 3064 3065 5470 5494 6462 5495 5471 6463 7632 8260 7633 8261 9184 9185
Route 3 :  306 307 960 848 849 961 2522 2523 2020 3198 2021 3199 4984 4985 6708 6709 6870 7704 7705 6871 8276 8277 8782 8783
Route 4 :  120 121 1546 2156 2157 1547 3112 3113 4036 4470 4037 4471 5150 5151 3766 6262 3767 6263 7956 7957 9106 9262 9107 9263
Route 5 :  364 462 463 365 2834 2835 3742 3743 5490 5491 6446 6447 8054 8055 8744 8745 9338 9339
```

Note that the Authors, Date and Reference information may not be set to meaningful values, as post-processing may not been applied on all solution files. Detailed solution information regarding routing decisions can be used to verify their structure. Note that the initial vehicle starting position (or depot in case of the PDPTW) is not included in the routes. Route numbering, however, correspond to vehicle ids (Route 1 -> Vehicle 1, etc.). Furthermore, no additional information on time or load is stored in the solution files, only the sequence of visits for each route.