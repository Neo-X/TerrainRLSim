// Implementation of the CMA-ES
#include <shark/Algorithms/DirectSearch/CMA.h>
// Access to benchmark functions
#include <shark/ObjectiveFunctions/Benchmarks/Benchmarks.h>


using namespace shark;

nt main(int argc, char **argv)
{
	// Adjust the floating-point format to scientific and increase output precision.
        cout.setf( ios_base::scientific );
        cout.precision( 10 );
    
        // Instantiate the problem.
        Sphere sphere( 2 );
        // Initialize the optimizer for the objective function instance.
        CMA cma;
        cma.init( sphere );
        cma.setSigma( 0.1 ); // Explicitely set initial globael step size.
    
        // Iterate the optimizer until a solution of sufficient quality is found.
        do {
            cma.step( sphere );
    
            // Report information on the optimizer state and the current solution to the console.
            cout << sphere.evaluationCounter() << " " << cma.solution().value << " " << cma.solution().point << " " << cma.sigma() << endl;
        } while(cma.solution().value > 1E-20 ); 

    return 0;
}