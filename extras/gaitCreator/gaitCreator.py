# Gait Creator !

step_user_input = "s"
draw_back_user_input = "db"
pause_user_input = "p"

gait_array = []

def get_code_index(lines, line):
  return lines[line].find("1")

def get_gait_code(leg_action):
  gait_scheduler_codes = open('lib/QuadrupedKinematics/src/gait-scheduler-codes.h', 'r')
  lines = gait_scheduler_codes.readlines()

  content_line = 0
  while lines[content_line] != "// --content--\n"  :
    content_line += 1
    if content_line > 50:
      print("What did you do to the content in gait-scheduler-codes.h?")
      print("Quiting")
      exit()


  step_code = int(lines[content_line + 1][lines[content_line + 1].find("1"): ].strip("\n"))
  draw_back_code = int(lines[content_line + 2][lines[content_line + 2].find("1"): ].strip("\n"))
  pause_code = int(lines[content_line + 3][lines[content_line + 3].find("1"): ].strip("\n"))

  if leg_action == step_user_input:
    return step_code
  elif leg_action == draw_back_user_input:
    return draw_back_code
  elif leg_action == pause_user_input:
    return pause_code


def print_user_codes():
  print(f"Select an action (step = '{step_user_input}', draw back = '{draw_back_user_input}', pause = '{pause_user_input}') for each of the 4 legs for every step.")

def time_until_first_step(strideDuration, schedule_index, leg_index, step_count):
  steps_until_first = schedule_index
  for leg in range(0, step_count):
    leg_action = gait_array[steps_until_first][leg_index]
    print(leg_action)
    if leg_action == get_gait_code(step_user_input):
      break
    else: 
      steps_until_first += 1
  
  return (steps_until_first - schedule_index) * strideDuration


def main():
  
  # Establishing the front matter (the basic parameters of the gait)
  gait_array.append(int(input("Enter the stride length ")))
  gait_array.append(int(input("Enter the gait amplitude ")))
  step_duration = int(input("Enter the duration of the gait's stride (millis) "))
  gait_array.append(step_duration)
  gait_array.append(int(input("Enter the multiplication factor of the amplitude during the draw back phase\n")))
  schedule_start_index = len(gait_array)

  print("Time to create the schedule! The wizard will allow you to select an action (step = 's', draw back = 'db', pause = 'p') for each of the 4 legs for every step.")
  print("If you forget a code, enter 'help' on any stage of the wizard.")
  print("If you finish, type 'done'.\n")


  # Enter in the gait
  step_number = 1
  schedule_complete = False
  while True:

    step = []

    print(f"In step {step_number}")

    for leg in range(0,4):
      leg_action_input = input(f"Enter the action for leg {leg + 1}\n").lower()

      if leg_action_input == "help":
        print_user_codes()
        leg -= 1
        leg_action_input = input(f"Enter the action for leg {leg}\n").lower()
      if leg_action_input == "done":
        schedule_complete = True
        step_number -= 1
        break
      elif leg_action_input == step_user_input or leg_action_input == draw_back_user_input or leg_action_input == pause_user_input:
        step.append(get_gait_code(leg_action_input))
      else:
        print(f"Error: {leg_action_input} is not a valid character.")
        exit()

    if schedule_complete:
      break

    step_number += 1
    gait_array.append(step)
  
  # Calculate the time until the first step for each leg 
  first_step_timings = []
  for leg in range(0, 4):
    first_step_timings.append(time_until_first_step(step_duration, schedule_start_index, leg, step_number))
  gait_array.insert(schedule_start_index, first_step_timings)

  gait_array_output = str(gait_array)
  gait_array_output = gait_array_output.replace("[", "{")
  gait_array_output = gait_array_output.replace("]", "}")
  print(gait_array_output)

if __name__ == "__main__":
  main()