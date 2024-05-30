import matplotlib.pyplot as plt

data_file = 'flight_data.txt'

# Initialize variables for storing results
results = {'R': [], 'L': []}
desired_gestures = {'R': [], 'L': []}
current_results = {'R': {}, 'L': {}}
current_desired = {'R': None, 'L': None}

# Read the data file
with open(data_file, 'r') as file:
    for line in file:
        line = line.strip()
        if line:
            parts = line.split()
            hand, actual_gesture, identified_gesture, _ = parts

            # Check for a new measurement group for the current hand
            if current_desired[hand] != actual_gesture:
                if any(current_results[hand]):
                    results[hand].append(current_results[hand])
                    desired_gestures[hand].append(current_desired[hand])
                    current_results[hand] = {}

                current_desired[hand] = actual_gesture  # Update the desired gesture at the start of a new group

            # Record results
            if identified_gesture not in current_results[hand]:
                current_results[hand][identified_gesture] = 0
            current_results[hand][identified_gesture] += 1

# Save the last group for each hand
for hand in ['R', 'L']:
    if any(current_results[hand]):
        results[hand].append(current_results[hand])
        desired_gestures[hand].append(current_desired[hand])

# Plotting results
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(18, 24))
bar_width = 0.35
opacity = 0.8
group_spacing = 1.0  # Adjust spacing for visual separation

for ax, hand in zip([ax1, ax2], ['R', 'L']):
    offset = 0
    for group_index, group in enumerate(results[hand]):
        positions = []
        base_position = offset
        for j, (identified_gesture, count) in enumerate(sorted(group.items())):
            pos = base_position + j
            positions.append(pos)
            color = 'green' if identified_gesture == desired_gestures[hand][group_index] else 'red'
            ax.bar(pos, count, bar_width, color=color, alpha=opacity)
            ax.text(pos, count, identified_gesture, ha='center', rotation=45)

        # Label and line drawing logic
        if positions:
            group_center = (positions[0] + positions[-1]) / 2
            ax.text(group_center, 0, f'Measurement {group_index + 1}', ha='center', va='top')
            if group_index < len(results[hand]) - 1:  # Don't draw a line after the last group
                ax.axvline(x=positions[-1] + 0.5 * group_spacing, color='gray', linestyle='--', linewidth=1)

        offset += len(group) + group_spacing  # Update offset with spacing for separation

    ax.set_xlabel('Gestures')
    ax.set_ylabel('Counts')
    ax.set_title(f'Gesture Recognition Results for {hand} Hand')

plt.tight_layout()
plt.show()
