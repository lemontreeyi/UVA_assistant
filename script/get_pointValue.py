
value_x = [
    0.872073,
    0.822221,
    0.870553,
    0.851976,
    0.837722,
    0.872216,
    0.885673,
    0.856934,
    0.859451,
    0.858328
]
value_y = [
4.212216,
4.112633,
4.211540,
4.179966,
4.182440,
4.201736,
4.222196,
4.169702,
4.195385,
4.187017
]
sum_x, sum_y = 0, 0
for i in value_x:
    sum_x += i
for i in value_y:
    sum_y += i
print(sum_x/10, sum_y/10)