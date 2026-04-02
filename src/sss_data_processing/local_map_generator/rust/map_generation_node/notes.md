// ! DELETE THIS FILE LATER

Will need a class
That holds V and P 
A function to buffer/generate data idk what to all it
A function to calulate the Map
Functions to return differnet data like Q_m, V_m, P_m ect...

// Buffering =============
// Q_m and P_m
// The stage 1 n 2 of gating then P_m and then stage 3 of gatting
Ok so then the way forwtad is first get the Pixel set Q_m based on geometric correction we have already from swath pricessing like r_port or r_std and also the r_max and also the theta. Then we calulate the P_m for each pixel and dixcard the weak ilumations

// V_m
Now that we have Q_m we project eacm pixel into slant space V_m[q] or more sepceific we just generate an empty array the size of Q_m basicaly V_m[q]. Then we calculate 4 r_m[q, i] for each pixel where they woudl land on slant range, NOT the center of teh pixel but the 4 coreners of the pixel we get each of the pixel corners so for each q we have 4 r_m, ie we get r_m[q, i], i=1...4. Thsi is becasue each edge might be a bit different in the intensity so better vaeraging them out than taking just the center as that might be less accurate.
Now each r_m[q, i] might not be perfcet aligned to each slant so we take the nearest 2 R slant measuremnts closest to r_m[q, i], ie the lowwer and upper bound of R(r_m[q, i]/resolution) and then interpolate those 2 pixels to get an aproximate strenght of that specific corner of teh pixel. that inetrpolated value is now 1 of 4 of our pixel corner. Now we do the same fo rthe other pixel corners by inetrpolating them r_m[q, i]. We are left with 4 instensities that represnet the pixels corners, to get the pixel istelf we just average it so sum all 4 r_m[q, i] intensity interpolations and divide by 4. Now we have V_m
Now do that process for the whole Q_m set

// V and P and chunk management
Now we have V_m and P_m for each pixel for THAT specific swath
Now we add this to the 2D map with other swath contributions V[q] and P[q] and thats what be buffer
IMPORTANT Here is where things need more management. V and P live in chunks like 64x64 or something like that. Each time a V_m and P_m is added to V and P that specific chunk in V and P gets renewed. Each V and P have AGING, and if they are not used for a long time those chunks will die in the future.
IF V and P chunk doesn't exist in the set we alocate memory for it and populate with 0 all and then add the V_m and P_m there, IMPORTANT to set AGE = 0 
IF V and P chunk exist now, we just add V_m and P_m, IMPORTANT to set AGE = 0 



// Map Calculation ==========
// M
IMPORTANT before we calculate M, we check age of each chunk in V and P, if that chunk is old ie AGE > AGE_LIMIT we dealocate it/delete it, otherwise we keep it in the set
Then finally we just calculate M map normalized
Then use kNN to fill in the gaps in M
IMPORTANT then we age all the V and P chunks AGE += 1



// NOTE:
The reaosn for aging and chunk management is so that V and P will otherwise grow with means when we clauclte M each time we will haev to reclauclate the whole map even teh old irelevant values. So for that we just Age chunks so old ones dont stick around if tehy are not used, savind unecesarry compute when we wnat M.


