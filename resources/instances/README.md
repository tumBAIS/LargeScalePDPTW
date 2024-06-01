### ``nyc``

Newly created instance set based on the New York City Taxi and Limousine Commission
Data ([link](https://www.nyc.gov/site/tlc/about/tlc-trip-record-data.page)).
Note that due to changes in the data (coordinates are no longer available), we used the processed data available
from https://github.com/tumBAIS/ML-CO-pipeline-AMoD-control. 

The instance files are compressed using [7-zip](https://7-zip.org/) and are located in `nyc/nyc.7z`. 
After extracting the archive the folder should have the following structure:

```
nyc/
|-- ny_2015_01_day-04/
    |-- 02h/
        |-- 60m/
            |-- ny_2015_01_day-04_02h_60m_b0m-do-fp_trips.csv
            |-- ny_2015_01_day-04_02h_60m_b0m-do-fp_v1000_c1.toml
            ...
        ...
        |-- ny_2015_01_day-04_02h_60m_vehicles_1000_cap1.csv
        ...
    |-- 06h/
    |-- 10h/
    |-- 14h/
    |-- 18h/
    |-- 22h/
|-- ny_2015_01_day-05/
|-- ny_2015_01_day-06/
|-- ny_2015_01_day-07/
|-- ny_2015_01_day-08/
|-- p25/
`-- nyc_graph_distances.csv
```

`ny_2015_01_day-04/` to `ny_2015_01_day-08/` contain the instances per weekday (Mon-Fri, Jan 5th-9th, 2015).
Each day contain six folders, one for each start of a time frame we considered.
Here, we store the data files regarding vehicles, e.g.,
`ny_2015_01_day-04_02h_60m_vehicles_1000_cap1.csv`, which contains 1000 vehicles (`*_vehicles_1000_*`), each with a
capacity of 1 (`*_cap1.*`)
Next, the folder `60m/`, which signifies the time frame, contain the trip data and configuration files.
E.g., `ny_2015_01_day-04_02h_60m_b0m-do-fp_trips.csv` contain the trip data files for the 60 minute time frame, with an
allowed buffer of `_b0m-*` of zero minutes.
Note that `fp` signifies that the pickup is fixed, i.e., the passenger must be picked up at the desired time, but may be
delivered later depending on the buffer setting.
Finally, the configuration files, e.g., `ny_2015_01_day-04_02h_60m_b0m-do-fp_v1000_c1.toml`, are the main entry files
for the
solver, and contain the complete information on the respective files needed for the instance, i.e., vehicle, trips, and
distance matrix file.

`p25/` contains the reduced, mid-sized instances generated for the computational analysis. The folder structure
follows the same logic as for the complete instances, however, only the time frame between 6-7pm was generated.

### `pdptw`

We do not include the instances for the pdptw benchmark set, which is available
to download via [Mendeley](https://data.mendeley.com/datasets/wr2ct4r22f/2).
For more information, we refer to the benchmark authors repository found
at https://github.com/cssartori/pdptw-instances.
