# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   
   1. Member function:
    ```
        #include <iostream>
        #include <cmath>

        struct Vector2D {
            double x = 0.0;
            double y = 0.0;

            // Member function to normalize the vector
            void normalize() {
                double mag = std::sqrt(x*x + y*y);

                // Check for division by zero to avoid NaN
                if (mag != 0.0) {
                    x /= mag;
                    y /= mag;
                }
            }
        };

        int main() {
            // Example vector
            Vector2D myVector {3.0, 4.0};

            // Normalize the vector
            myVector.normalize();

            // Display the normalized vector
            std::cout << "Normalized Vector (Design 1): (" << myVector.x << ", " << myVector.y << ")\n";

            return 0;
        }

    ```

   2. Free Function:

    ```
        #include <iostream>
        #include <cmath>

        struct Vector2D {
            double x = 0.0;
            double y = 0.0;
        };

        // Free function to normalize a vector
        Vector2D normalizeVector(const Vector2D& v) {
            double mag = std::sqrt(v.x*v.x + v.y*v.y);

            // Check for division by zero to avoid NaN
            if (mag != 0.0) {
                return {v.x / mag, v.y / mag};
            }

            return v; // Return original vector if magnitude is zero
        }

        int main() {
            // Example vector
            Vector2D myVector {3.0, 4.0};

            // Normalize the vector
            Vector2D normalizedVector = normalizeVector(myVector);

            // Display the normalized vector
            std::cout << "Normalized Vector (Design 2): (" << normalizedVector.x << ", " << normalizedVector.y << ")\n";

            return 0;
        }

    ```

    3. Operator Overloading
    ```
        #include <iostream>
        #include <cmath>

        struct Vector2D {
            double x = 0.0;
            double y = 0.0;

            // Operator overloading to normalize the vector
            Vector2D operator/(double scalar) const {
                // Check for division by zero to avoid NaN
                if (scalar != 0.0) {
                    return {x / scalar, y / scalar};
                }

                return *this; // Return original vector if scalar is zero
            }

            // Member function to calculate the magnitude of the vector
            double magnitude() const {
                return std::sqrt(x*x + y*y);
            }

            // Member function to normalize the vector
            Vector2D normalize() const {
                double mag = magnitude();

                // Use the overloaded division operator
                return *this / mag;
            }
        };

        int main() {
            // Example vector
            Vector2D myVector {3.0, 4.0};

            // Normalize the vector
            Vector2D normalizedVector = myVector.normalize();

            // Display the normalized vector
            std::cout << "Normalized Vector (Design 3): (" << normalizedVector.x << ", " << normalizedVector.y << ")\n";

            return 0;
        }

    ```

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

   - Which of the methods would you implement and why?

2. What is the difference between a class and a struct in C++?


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
