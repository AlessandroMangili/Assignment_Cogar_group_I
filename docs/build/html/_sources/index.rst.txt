:cogarmodule: Cogar Assignment - Group I
:cogardate: April 2025

=============================
COGAR ASSIGNMENT - TOPIC 3
=============================

Assisting elderly individuals in meal preparation
----------------------------------------------------

**Platform:** TIAGo robot deployed in a kitchen environment to assist elderly people with cooking tasks. The robot system is equipped with:

- RGB-D Camera
- LiDAR
- SONAR
- Force Sensors
- Microphones
- Speakers

**Scope of Work:**

1. **Ingredient retrieval**  
   - Plan and execute efficient, obstacle-free paths using vision and real-time mapping to fetch ingredients.  
2. **Collaborative cooking process**  
   - Follow recipe steps (mixing, cutting, pouring), maintain an internal representation of progress, and update dynamically.  
3. **Action decision-making**  
   - Determine next actions based on task history and current state; verify and interpret verbal commands.  
4. **Verbal command handling**  
   - Resolve conflicts between user instructions and planned sequence by deciding to override or reject.  
5. **Object recognition**  
   - Identify and locate tools and ingredients in the workspace.  
6. **Mission completion**  
   - Confirm completion of all recipe steps and notify the user when the meal is ready.

Mandatory Components
----------------------

- **Recipe tracking and execution history**  
  Maintains a dynamic model of the recipe sequence and logs all performed actions.

- **Action planning based on cooking state and task history**  
  Chooses the next best action by analyzing current progress and past steps, adapting to unexpected conditions.

- **Human command monitoring and conflict resolution**  
  Interprets user verbal commands, validates or rejects them against the planned workflow, and updates the plan accordingly.

.. toctree::
    :maxdepth: 2
    :caption: Indices:

    Components <components.rst>
    Behevioural <behavioural.rst>
    Interfaces <interfaces.rst>
    Testing and KPIs <testing.rst>
