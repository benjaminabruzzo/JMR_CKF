Rk = [...
 10000.0000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
 0.00000000    10000.0000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
 0.00000000    0.00000000    10000.0000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
 0.00000000    0.00000000    0.00000000    10000.0000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
 0.00000000    0.00000000    0.00000000    0.00000000    0.00000930   -0.00006884   -0.00000060    0.00000000 ; ...
 0.00000000    0.00000000    0.00000000    0.00000000   -0.00006884    0.00059441    0.00000433    0.00000000 ; ...
 0.00000000    0.00000000    0.00000000    0.00000000   -0.00000060    0.00000433    0.00000451    0.00000000 ; ...
 0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000015 ; ...
  ];

xk = [ 0.0000000000;  0.0000000000;  0.0000000000;  0.0000000000;  0.0000000000;  0.0000000000;  0.0000000000;  0.0000000000;  0.0000000000;  0.0000000000;  0.0000000000;  0.0000000000;       ];
zk = [ 0.0000000000;  0.0000000000;  0.0000000000;  0.0000000000;  -0.0005364657;  0.0038576139;  -0.0052400572;  -0.4772171726;       ]

H = [...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000   -1.00000000    0.00000000    0.00000000    0.00000000    1.00000000    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000   -1.00000000    0.00000000    0.00000000    0.00000000    1.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000   -1.00000000    0.00000000    0.00000000    0.00000000    1.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000   -1.00000000    0.00000000    0.00000000    0.00000000    1.00000000 ; ...
]

PriorCov = [...
10.13830000   0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    10.13830000   0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    10.13830000   0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    10.13830000   0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.03871545    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.02334930    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00100000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.09332455    0.00000000    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00100082    0.00000000    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00100418    0.00000000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00100000    0.00000000 ; ...
0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.00000000    0.01654136 ; ...
];

Sk = H * PriorCov * H' + Rk
Skinv = inv(Sk)

yk = zk - H*xk

Kk = PriorCov * H' * Skinv
Kkyk = Kk * yk

Kk * Rk * Kk'

