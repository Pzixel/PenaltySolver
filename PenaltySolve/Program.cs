using System;
using System.Collections.Generic;
using System.Linq;
using Model;

namespace PenaltySolve
{
    public class Program
    {
        private static void Main()
        {
            Func<double[], double> f = x => Math.Pow(x[0] - 2, 4) + Math.Pow(x[0] - 2*x[1], 2);
            Func<double[], double> penalty = x => x[0]*x[0] - x[1];
            double[] range;
            while (true)
            {
                Console.WriteLine("Введите координаты начальной точки через пробел");
                try
                {
                    range = Console.ReadLine().Split(' ').Select(double.Parse).ToArray();
                    if (range.Length == 2)
                        break;
                }
                catch {}
            }
            double epsilon, mu;
            int beta;
            do Console.WriteLine("Введите точность (либо с точкой, либо с запятой)"); while (!double.TryParse(Console.ReadLine(), out epsilon));
            do Console.WriteLine("Введите значение Mu"); while (!double.TryParse(Console.ReadLine(), out mu));
            do Console.WriteLine("Введите значение Beta"); while (!int.TryParse(Console.ReadLine(), out beta));
            Console.WriteLine("Начата работа из точки [{0};{1}] с точностью {2} и начальным значением Mu = {3}", range[0], range[1], epsilon, mu);
            
            var p = new PenaltySolver(f, penalty, range, epsilon, mu, beta);
            var res = p.Solve();
            Console.WriteLine(" Mu\t   Xmu\t\tF(mu) Alpha Theta MuAlpha");
            foreach (var d in p.Details)
                Console.WriteLine("{0,4} {1}\t{2:f}  {3:f}  {4:f}  {5:f}", d.Mu, d.X, d.Y, d.Alpha, d.Theta, d.MuAlpha);
            Console.WriteLine("\n\nX = [{0:N6};{1:N6}]\t\tF(X) = {2}\n\n", res[0], res[1], f(res));
            Console.ReadKey();
        }
    }


    public class PenaltySolver
    {
        private readonly Func<double[], double> f;
        private readonly Func<double[], double> penalty;
        private readonly Vector startPoint;
        private readonly double epsilon;
        private double mu;
        private readonly double beta;
        private List<PenaltySolverDetails> details;

        public IEnumerable<PenaltySolverDetails> Details { get { return details.Select(x => x); } }

        private double Function(double[] x)
        {
            return f(x) + Penalty(x);
        }

        private double Penalty(double[] x)
        {
            return Math.Max(mu*penalty(x), 0);
        }

        public PenaltySolver(Func<double[], double> f, Func<double[], double> penalty, Vector startPoint, double epsilon,
                             double mu, double beta)
        {
            if (f == null || penalty== null || epsilon <= 0 || mu <= 0 || beta <= 1)
                throw new ArgumentException();
            this.f = f;
            this.penalty = penalty;
            this.startPoint = startPoint;
            this.epsilon = epsilon;
            this.mu = mu;
            this.beta = beta;
            SmartPoint.SetFunction(f);
        }

        public Vector Solve()
        {
            details = new List<PenaltySolverDetails>();
            Vector x = startPoint;
            while (true)
            {
                var r = new Rosenbrock(Function, x, epsilon);
                Vector oldx = x;
                x = r.Solve();
                details.Add(new PenaltySolverDetails(mu, new SmartPoint(oldx), Math.Pow(Penalty(x), 2)));
                if (details.Last().MuAlpha < epsilon)
                    break;
                mu *= beta;
            }
            return x;
        }
    }

    public class PenaltySolverDetails
    {
        public double Mu { get; private set; }
        public Vector X { get; private set; }
        public double Y { get; private set; }
        public double Alpha { get; private set; }
        public double Theta { get; private set; }
        public double MuAlpha { get; private set; }

        public PenaltySolverDetails(double mu, SmartPoint smart, double alpha)
        {
            Mu = mu;
            X = smart.Coordinates;
            Y = smart.FunctionValue;
            Alpha = alpha;
            Theta = alpha + Y;
            MuAlpha = mu * alpha;
        }
    }
}