# tools
A set of tools for my research.


# Compilation
1. install the famous IDE DEV-C++
2. download the Eigen package
3. open dic_tools.dev using DEV-C++
4. Press F12 to rebuild the project


# Usage(Windows CMD)
```
  .\dic_tools .\sample\data.txt
```
  or
```
  .\dic_tools .\sample\data.txt e                //then input the ransac distance threshold
```

Then run the python to display the 3d points scatter diagrams.
```
  cd sample
  python draw.py data.txt.transed.txt
  
```
