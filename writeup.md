##  CarND-Controls-PID

[//]: # (Image References)
[image1]: ./results/Lecture_P.png
[image2]: ./results/Lecture_PD.png
[image3]: ./results/PD_error.png
[image4]: ./results/PID_error.png

[video1]: ./results/001_P2.mov
[video2]: ./results/002_P2.mov
[video3]: ./results/003_P2.mov

---

## [Rubric](https://review.udacity.com/#!/rubrics/824/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup

---

### Compilation

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

---

### Implementation
The PID procedure follows what was taught in the lessons.

---

### Reflection
#### Describe the effect each of the P, I, D components had in your implementation.
##### P component
As fisrt, I used only P components. P = 0.1.
When the car was running, oscillation became bigger and bigger, and then finally car went out from the course.
As explained in the lecture, using only P component made oscillation.

[P component](./results/001_P2.mov)

![alt text][image1]


##### P and D components
Second, I used P and D components. P = 0.1, D = 1.0. 
As explained in the lecture, D component improved oscillation.

[PD component](./results/002_PD2.mov)

![alt text][image2]

##### P.I.D components
Finally, I used P,I,D components. P = 0.1, D = 1.0 and I = 0.0001. 
When I compared the total CTE and PID error, PID was better than PD. So this means that PID made car run closer to the target line of track, and bias of PID was better than bias of PI.

[PID component](./results/003_PID2.mov)

![alt text][image3]
![alt text][image4]

| Components    | Total CTE (err) | Total PID error
|:-------------:|:---------------:|:---------------:|
| PD            | 415.061         | 536.334
| PID         	| 219.873         | 321.041 

#### Describe how the final hyperparameters were chosen.
I implemented Twiddle method that explained in the lecture, and acitivated Twiddle.

```C++
// update pid error
pid.UpdateError(cte);
// Twiddle? uncomment below
pid.Twiddle_PID(cte);
```

After a few hours running, I got following parameters from Twiddle.

| Components    | Value
|:-------------:|:---------------:|
| P             | 0.198258
| I             | 0.000115576
| D             | 1.78716

---

### Simulation
#### The vehicle must successfully drive a lap around the track.
Car should run continuously with above parameters.

