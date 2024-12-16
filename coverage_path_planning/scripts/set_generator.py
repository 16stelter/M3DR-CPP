import random

numbers = list(range(1, 1001))
line_counter = 1

while numbers:
    result_line = [line_counter]
    
    num_to_pick = random.randint(1, len(numbers))
    picked_numbers = random.sample(numbers, num_to_pick)
    result_line.extend(picked_numbers)
    result_line.append(-1)
    print(" ".join(map(str, result_line)))
    for num in picked_numbers:
        numbers.remove(num)
    line_counter += 1
