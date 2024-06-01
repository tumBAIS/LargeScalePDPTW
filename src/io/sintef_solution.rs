use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Lines, Write};

use crate::problem::pdptw::PDPTWInstance;
use crate::solution::SolutionDescription;
use crate::solver::SolverResult;

/**
From the README.txt in the data set from https://data.mendeley.com/datasets/wr2ct4r22f/2; in the /solutions folder

----------

## Structure of solution files.

"Instances for the Pickup and Delivery Problem with Time Windows
 based on open data"

===============================================================================
===============================================================================

Solution files follow the same structure from the SINTEF TOP website

https://www.sintef.no/projectweb/top/pdptw/

The file contains a header with four information fields:

Instance name:    <name of the instance>
Authors:          <name of the authors who found the solution>
Date:             <date when solution was found yyyy-mm-dd>
Reference:        <a reference to the work with title or report number>

This is followed by a line with the word "Solution", and then
as many lines as there are routes in this solution, detailing every
vehicle route. This is done by using the structure

        Route X : n1 n2 n3 n4 ... nk

Where X is the index of the route and n1,n2,n3,n4,nk are the nodes
of this route, which contain k locations.

NOTE: the depot is not included in the reporting of the routes.


We also follow the naming convention of SINTEF for solution files.
That is, a solution with V vehicles and cost C for the instance file
"inst.txt" will have the name

     inst.V_C.txt

===============================================================================
===============================================================================

EXAMPLE:
    As an example, consider the following solution with 6 routes.


Instance name:    bar-n100-1
Authors:          Carlo Sartori and Luciana Buriol
Date:             2019-2-11
Reference:        A matheuristic approach to the PDPTW (to be submitted).
Solution
Route 1 : 31 44 35 81 16 66 32 82 19 85 94 69
Route 2 : 29 21 71 27 47 79 11 22 97 77 72 6 61 25 56 75 1 51
Route 3 : 40 48 9 59 5 55 90 98 41 8 10 91 60 38 28 78 88 58
Route 4 : 26 76 30 80 7 39 57 42 92 12 89 18 62 68 37 36 87 50 100 86
Route 5 : 14 15 64 49 45 43 65 4 99 46 95 96 54 93 23 73
Route 6 : 33 13 63 20 83 17 67 2 34 52 84 24 70 74 3 53



This solution file is named:

     bar-n100-1.6_733.txt

===============================================================================
===============================================================================

## Carlo Sartori and Luciana Buriol (2019).
 */
pub fn load_sintef_solution(path: impl Into<String>) -> anyhow::Result<SINTEFSolution> {
    let f = File::open(path.into())?;
    let file = BufReader::new(&f);

    let mut lines = file.lines();

    let (instance_name, authors, date_string, reference) = read_solution_info(&mut lines)?;
    let routes = read_routes(&mut lines)?;

    Ok(SINTEFSolution {
        instance_name,
        authors,
        date_string,
        reference,
        routes,
    })
}

pub struct SINTEFSolution {
    /// Instance name:    bar-n100-1
    pub instance_name: String,
    /// Authors:          Carlo Sartori and Luciana Buriol
    pub authors: String,
    /// Date:             2019-2-11
    pub date_string: String,
    /// Reference:        A matheuristic approach to the PDPTW (to be submitted).
    pub reference: String,
    /// Solution - List of itineraries with node ids (no depot node)
    pub routes: Vec<Vec<usize>>,
}

fn read_solution_info(
    lines: &mut Lines<BufReader<&File>>,
) -> anyhow::Result<(String, String, String, String)> {
    // Instance name:    bar-n100-1
    let instance_name = parse_property_value_string(lines.next().unwrap()?)?;
    // Authors:          Carlo Sartori and Luciana Buriol
    let authors = parse_property_value_string(lines.next().unwrap()?)?;
    // Date:             2019-2-11
    let date_string = parse_property_value_string(lines.next().unwrap()?)?;
    // Reference:        A matheuristic approach to the PDPTW (to be submitted).
    let reference = parse_property_value_string(lines.next().unwrap()?)?;

    Ok((instance_name, authors, date_string, reference))
}

fn parse_property_value_string(line: String) -> anyhow::Result<String> {
    let mut split = line.split(":");
    split.next().unwrap(); // discard the first part
    Ok(split.next().unwrap().trim().to_string())
}

fn read_routes(lines: &mut Lines<BufReader<&File>>) -> anyhow::Result<Vec<Vec<usize>>> {
    // Solution
    lines.next().unwrap()?;

    let mut routes = vec![];
    for line in lines
        .filter_map(|it| it.ok())
        .filter(|it| !it.trim().is_empty())
    {
        let mut split = line.split(":");
        split.next().unwrap(); // discard the first part
        let itinerary = split.next().unwrap().trim();
        let mut route = vec![];
        if itinerary.is_empty() {
            routes.push(vec![]);
        } else {
            for id in itinerary.split(" ") {
                route.push(id.parse()?);
            }
            route.shrink_to_fit();
            routes.push(route);
        }
    }

    Ok(routes)
}

pub struct SINTEFSolutionBuilder {
    pub instance_name: Option<String>,
    pub authors: Option<String>,
    pub date_string: Option<String>,
    pub reference: Option<String>,
    pub routes: Vec<Vec<usize>>,
}

impl SINTEFSolutionBuilder {
    pub fn new() -> Self {
        Self {
            instance_name: None,
            authors: None,
            date_string: None,
            reference: None,
            routes: vec![],
        }
    }
    pub fn instance_name(&mut self, instance_name: impl Into<String>) -> &mut Self {
        self.instance_name = Some(instance_name.into());
        self
    }
    pub fn authors(&mut self, authors: impl Into<String>) -> &mut Self {
        self.authors = Some(authors.into());
        self
    }
    pub fn date_string(&mut self, date_string: impl Into<String>) -> &mut Self {
        self.date_string = Some(date_string.into());
        self
    }
    pub fn reference(&mut self, reference: impl Into<String>) -> &mut Self {
        self.reference = Some(reference.into());
        self
    }
    pub fn routes(&mut self, routes: Vec<Vec<usize>>) -> &mut Self {
        self.routes = routes;
        self
    }
    pub fn routes_from_solution_description(
        &mut self,
        description: &SolutionDescription,
        instance: &PDPTWInstance,
    ) -> &mut Self {
        self.routes.clear();
        let mut routes = description.to_routes_vec(instance);
        routes.sort_by_key(|it| it[0]);
        for route in routes.iter_mut() {
            route.pop();
            route.remove(0);
            for node in route.iter_mut() {
                *node = instance.nodes[*node].oid
            }
        }
        self.routes = routes;
        self
    }
    pub fn build(self) -> SINTEFSolution {
        SINTEFSolution {
            instance_name: self.instance_name.unwrap_or("UNKNOWN".to_string()),
            authors: self.authors.unwrap_or("Gerhard Hiermann".to_string()),
            reference: self.reference.unwrap_or("XYZ".to_string()),
            date_string: self.date_string.unwrap_or("2022-mm-dd".to_string()),
            routes: self.routes,
        }
    }
}

pub fn write_sintef_solution(
    path: impl Into<String>,
    solution: SINTEFSolution,
    result: Option<SolverResult>,
) -> anyhow::Result<()> {
    let f = File::create(path.into())?;
    let mut file = BufWriter::new(&f);

    // Instance name:    bar-n100-1
    writeln!(file, "Instance name:    {}", solution.instance_name)?;
    // Authors:          Carlo Sartori and Luciana Buriol
    writeln!(file, "Authors:          Gerhard Hiermann")?;
    // Date:             2019-2-11
    writeln!(file, "Date:             2022-mm-dd")?;
    // Reference:        A matheuristic approach to the PDPTW (to be submitted).
    writeln!(file, "Reference:        XYZ")?;
    // Solution
    if let Some(res) = result {
        writeln!(
            file,
            "Solution: {};{};{};{}",
            res.solution.number_of_unassigned_requests(),
            res.solution.vehicles_used,
            res.solution.objective,
            res.time
        )?;
    } else {
        writeln!(file, "Solution")?;
    }

    #[cfg(feature = "classic-pdptw")]
        let iter = solution.routes.iter().filter(|route| route.len() > 0);
    #[cfg(not(feature = "classic-pdptw"))]
        let iter = solution.routes.iter();

    // Route 1 : 31 44 35 81 16 66 32 82 19 85 94 69
    // Route 2 : 29 21 71 27 47 79 11 22 97 77 72 6 61 25 56 75 1 51
    // ...
    for (idx, route) in iter.enumerate() {
        write!(file, "Route {} : ", idx + 1)?;
        for node in route {
            write!(file, " {}", node)?;
        }
        writeln!(file, "")?;
    }

    Ok(())
}

#[cfg(feature = "test-with-sartoriburiol-solutions")]
#[cfg(test)]
pub mod tests {
    use super::*;

    #[test]
    fn read_100_solution() -> anyhow::Result<()> {
        for (_, solution_path, _, _) in sartori_buriol::N100 {
            load_sintef_solution(format!(
                "{}/{}",
                sartori_buriol::SOLUTION_DIR_N100,
                solution_path
            )).expect("Instance not loaded");
        }
        Ok(())
    }

    pub mod sartori_buriol {
        pub const INSTANCE_BASE_DIR: &str = "resources/instances/SartoriBuriol/";
        pub const INSTANCE_DIR_N100: &str = "resources/instances/SartoriBuriol/n100/";

        pub const SOLUTION_BASE_DIR: &str = "resources/solutions/SartoriBuriol/";
        pub const SOLUTION_DIR_N100: &str = "resources/solutions/SartoriBuriol/n100/";

        pub const N100: [(&'static str, &'static str, usize, usize); 25] = [
            ("bar-n100-1.txt", "bar-n100-1.6_733.txt", 6, 733),
            ("bar-n100-2.txt", "bar-n100-2.5_554.txt", 5, 554),
            ("bar-n100-3.txt", "bar-n100-3.6_746.txt", 6, 746),
            ("bar-n100-4.txt", "bar-n100-4.12_1154.txt", 12, 1154),
            ("bar-n100-5.txt", "bar-n100-5.6_838.txt", 6, 838),
            ("bar-n100-6.txt", "bar-n100-6.3_788.txt", 3, 788),
            ("ber-n100-1.txt", "ber-n100-1.13_1857.txt", 13, 1857),
            ("ber-n100-2.txt", "ber-n100-2.6_1491.txt", 6, 1491),
            ("ber-n100-3.txt", "ber-n100-3.3_713.txt", 3, 713),
            ("ber-n100-4.txt", "ber-n100-4.3_494.txt", 3, 494),
            ("ber-n100-5.txt", "ber-n100-5.5_944.txt", 5, 944),
            ("ber-n100-6.txt", "ber-n100-6.14_2147.txt", 14, 2147),
            ("ber-n100-7.txt", "ber-n100-7.7_1935.txt", 7, 1935),
            ("nyc-n100-1.txt", "nyc-n100-1.6_634.txt", 6, 634),
            ("nyc-n100-2.txt", "nyc-n100-2.4_567.txt", 4, 567),
            ("nyc-n100-3.txt", "nyc-n100-3.3_492.txt", 3, 492),
            ("nyc-n100-4.txt", "nyc-n100-4.2_535.txt", 2, 535),
            ("nyc-n100-5.txt", "nyc-n100-5.2_671.txt", 2, 671),
            ("poa-n100-1.txt", "poa-n100-1.12_1589.txt", 12, 1589),
            ("poa-n100-2.txt", "poa-n100-2.15_1539.txt", 15, 1539),
            ("poa-n100-3.txt", "poa-n100-3.10_1301.txt", 10, 1301),
            ("poa-n100-4.txt", "poa-n100-4.7_1668.txt", 7, 1668),
            ("poa-n100-5.txt", "poa-n100-5.6_624.txt", 6, 624),
            ("poa-n100-6.txt", "poa-n100-6.3_562.txt", 3, 562),
            ("poa-n100-7.txt", "poa-n100-7.5_779.txt", 5, 779),
        ];

        pub const INSTANCE_DIR_N200: &str = "resources/instances/SartoriBuriol/n200/";
        pub const SOLUTION_DIR_N200: &str = "resources/solutions/SartoriBuriol/n200/";

        pub const N200: [(&'static str, &'static str, usize, usize); 25] = [
            ("bar-n200-1.txt", "bar-n200-1.22_1829.txt", 22, 1829),
            ("bar-n200-2.txt", "bar-n200-2.23_2072.txt", 23, 2072),
            ("bar-n200-3.txt", "bar-n200-3.8_1644.txt", 8, 1644),
            ("bar-n200-4.txt", "bar-n200-4.13_838.txt", 13, 838),
            ("bar-n200-5.txt", "bar-n200-5.5_854.txt", 5, 854),
            ("bar-n200-6.txt", "bar-n200-6.9_855.txt", 9, 855),
            ("bar-n200-7.txt", "bar-n200-7.11_1901.txt", 11, 1901),
            ("ber-n200-1.txt", "ber-n200-1.28_3189.txt", 28, 3189),
            ("ber-n200-2.txt", "ber-n200-2.12_3265.txt", 12, 3265),
            ("ber-n200-3.txt", "ber-n200-3.9_899.txt", 9, 899),
            ("ber-n200-4.txt", "ber-n200-4.5_1084.txt", 5, 1084),
            ("ber-n200-5.txt", "ber-n200-5.27_3944.txt", 27, 3944),
            ("ber-n200-6.txt", "ber-n200-6.9_3016.txt", 9, 3016),
            ("nyc-n200-1.txt", "nyc-n200-1.7_943.txt", 7, 943),
            ("nyc-n200-2.txt", "nyc-n200-2.8_1104.txt", 8, 1104),
            ("nyc-n200-3.txt", "nyc-n200-3.7_1019.txt", 7, 1019),
            ("nyc-n200-4.txt", "nyc-n200-4.4_1037.txt", 4, 1037),
            ("nyc-n200-5.txt", "nyc-n200-5.5_1193.txt", 5, 1193),
            ("poa-n200-1.txt", "poa-n200-1.25_2433.txt", 25, 2433),
            ("poa-n200-2.txt", "poa-n200-2.13_2347.txt", 13, 2347),
            ("poa-n200-3.txt", "poa-n200-3.22_1850.txt", 22, 1850),
            ("poa-n200-4.txt", "poa-n200-4.10_1163.txt", 10, 1163),
            ("poa-n200-5.txt", "poa-n200-5.15_2321.txt", 15, 2321),
            ("poa-n200-6.txt", "poa-n200-6.27_3160.txt", 27, 3160),
            ("poa-n200-7.txt", "poa-n200-7.11_2463.txt", 11, 2463),
        ];

        pub const INSTANCE_DIR_N400: &str = "resources/instances/SartoriBuriol/n400/";
        pub const SOLUTION_DIR_N400: &str = "resources/solutions/SartoriBuriol/n400/";

        pub const N400: [(&'static str, &'static str, usize, usize); 1] =
            [("nyc-n400-1.txt", "nyc-n400-1.13_1947.txt", 13, 1947)];

        pub const INSTANCE_DIR_N1000: &str = "resources/instances/SartoriBuriol/n1000/";
        pub const SOLUTION_DIR_N1000: &str = "resources/solutions/SartoriBuriol/n1000/";

        pub const N1000: [(&'static str, &'static str, usize, usize); 1] =
            [("nyc-n1000-1.txt", "nyc-n1000-1.27_4013.txt", 27, 4013)];
    }
}
