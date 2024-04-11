# Multi-Objective Search

This repo includes the implementations for various multi-objective search algorithms:

+ BOA* [1]
+ ABOA* [2]
+ LTMOA* [3] and its variants with different data structures [4]
+ A*pex [5]
+ Anytime A*pex [6]
+ WCA*pex [7]

I am working on more detailed documentation and will update it soon.

## Compilation
```
mkdir Release
cd Release
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

## Example Usage

Download maps from [here](https://drive.google.com/file/d/1U_E9un7jOV1qNOnizFCqG-HPKHonx2X5/view?usp=share_link) 

Run LTMOA (from ./Release)
```
./mohs -m ../maps/NY-m.txt ../maps/NY-t.txt  ../maps/NY-d.txt --output output_tmp.txt -t 300 -s 178689 -g 1476 --alg LTMOA
```

Run A*pex
```
./mohs -m ../maps/NY-m.txt ../maps/NY-t.txt  ../maps/NY-d.txt --output output_tmp.txt -t 300 -s 178689 -g 1476 --alg Apex
```


## References
[1] Hernández, Carlos, et al. "Simple and efficient bi-objective search algorithms via fast dominance checks." Artificial intelligence 314 (2023): 103807.

[2] Zhang, Han, et al. "Anytime approximate bi-objective search." SoCS. 2022.

[3] Hernández, Carlos, et al. "Multi-objective search via lazy and efficient dominance checks." IJCAI. 2023.

[4] Zhang, Han, et al. "Speeding Up Dominance Checks in Multi-Objective Search: New Techniques and Data Structures" SoCS. 2024.

[5] Zhang, Han, et al. "A* pex: Efficient approximate multi-objective search on graphs." ICAPS. 2022.

[6] Zhang, Han, et al. "A-A*pex: Efficient Anytime Approximate Multi-Objective Search." SoCS. 2024.

[7] Zhang, Han, et al. "Bounded-Suboptimal Weight-Constrained Shortest-Path Search via Efficient Representation of Paths" ICAPS. 2024.

## Licence
[AGPL-3.0](https://opensource.org/licenses/AGPL-3.0)

