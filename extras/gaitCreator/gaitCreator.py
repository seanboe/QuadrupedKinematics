# Gait Creator !

def parse_leg_action(user_input):
  if user_input == "db":
    pass

# Gait Codes:
step = 0x0A
draw_back = 0x0B
pause = 0x0C

step_user_input = "s"
draw_back_user_input = "db"

gait_array = []

gait_array.append(int(input("Enter the stride length")))
gait_array.append(int(input("Enter the gait amplitude")))
gait_array.append(int(input("Enter the duration of the gait's stride (millis) ")))
gait_array.append(int(input("Enter the multiplication factor of the amplitude during the draw back phase\n")))
frontmatter_end = len(gait_array) - 1

print("End of global gait parameters; time to enter in the gait schedule.\n")
print(f"To denote the stepping of a leg, enter '{step_user_input}'\n")
print(f"To denote the drawback phase of a step, enter '{draw_back_user_input}'\n")

step_number = 1
while True:
  print(f"In step {step_number}")
  for leg in range(0,4):
    leg_action_input = input(f"Enter the action for leg {leg}")
    break