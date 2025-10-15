import os
import sys
from animation import run_single_simulation
from testing import run_tests_from_csv
from data_generator import generate_test_csv, create_sample_test_file, create_comprehensive_test_file

def clear_screen():
    """Clear the terminal screen"""
    os.system('cls' if os.name == 'nt' else 'clear')

def print_menu():
    """Print the main menu options"""
    print("🚗 Moving Car Simulation & Testing")
    print("=" * 40)
    print("1. Run Single Simulation")
    print("2. Run Tests from CSV File")
    print("3. Generate Test CSV File")
    print("4. Exit")
    print("=" * 40)

def get_menu_choice():
    """Get user's menu choice"""
    try:
        choice = int(input("Enter your choice (1-4): "))
        return choice
    except ValueError:
        print("Please enter a valid number!")
        return None

def select_file_manually():
    """Manual file selection with directory browsing"""
    base_dir = "test_configs" if os.path.exists("test_configs") else "."
    
    while True:
        print(f"\n📁 Current directory: {os.path.abspath(base_dir)}")
        print("Available CSV files:")
        
        # List CSV files
        csv_files = []
        try:
            for file in os.listdir(base_dir):
                if file.lower().endswith('.csv'):
                    csv_files.append(file)
                    print(f"  - {file}")
        except FileNotFoundError:
            print("  No CSV files found in directory")
        
        if not csv_files:
            print("No CSV files found.")
            choice = input("\nEnter file path (or 'b' to browse, 'q' to quit): ").strip()
        else:
            print("\nOptions:")
            print("  - Enter filename from list above")
            print("  - Enter full file path")
            print("  - 'd' to use default sample file")
            print("  - 'b' to browse different directory")
            print("  - 'q' to quit to main menu")
            choice = input("\nYour choice: ").strip()
        
        if choice.lower() == 'q':
            return None
        elif choice.lower() == 'd':
            return "test_configs/sample_tests.csv"
        elif choice.lower() == 'b':
            new_dir = input("Enter directory path: ").strip()
            if os.path.isdir(new_dir):
                base_dir = new_dir
            else:
                print("❌ Invalid directory")
        elif choice:
            # Check if it's a filename in current directory
            if os.path.isfile(os.path.join(base_dir, choice)):
                return os.path.join(base_dir, choice)
            # Check if it's a full path
            elif os.path.isfile(choice):
                return choice
            else:
                print("❌ File not found. Please try again.")
        else:
            print("❌ Please enter a valid option.")

def get_csv_file():
    """Get CSV file path using manual selection"""
    print("\n📂 CSV File Selection")
    print("=" * 30)
    
    # Check if default file exists
    if os.path.exists("test_configs/sample_tests.csv"):
        print("Default file available: test_configs/sample_tests.csv")
        use_default = input("Use default file? (y/n): ").strip().lower()
        if use_default == 'y':
            return "test_configs/sample_tests.csv"
    
    # Manual file selection
    return select_file_manually()

def wait_for_user():
    """Wait for user to press Enter before continuing"""
    print("\n" + "="*50)
    input("Press Enter to return to main menu...")

def main():
    """Main function with menu system"""
    # Create necessary directories
    os.makedirs("test_configs", exist_ok=True)
    os.makedirs("test_results", exist_ok=True)
    os.makedirs("test_results/plots", exist_ok=True)
    
    # Create sample test file if it doesn't exist
    if not os.path.exists("test_configs/sample_tests.csv"):
        create_sample_test_file()
        print("✅ Created sample test file: test_configs/sample_tests.csv")
    
    # Create comprehensive test file if it doesn't exist
    if not os.path.exists("test_configs/comprehensive_tests.csv"):
        create_comprehensive_test_file()
        print("✅ Created comprehensive test file: test_configs/comprehensive_tests.csv")
    
    while True:
        clear_screen()
        print_menu()
        choice = get_menu_choice()
        
        if choice == 1:
            # Run single simulation
            run_single_simulation()
            wait_for_user()
            
        elif choice == 2:
            # Run tests from CSV with file selector
            print("\n" + "="*50)
            print("SELECT TEST CSV FILE")
            print("="*50)
            
            file_path = get_csv_file()
            
            if file_path and os.path.exists(file_path):
                print(f"✅ Selected file: {file_path}")
                input("Press Enter to run tests...")
                run_tests_from_csv(file_path)
                wait_for_user()
            else:
                if file_path is None:
                    print("❌ File selection cancelled.")
                else:
                    print(f"❌ File not found: {file_path}")
                wait_for_user()
                
        elif choice == 3:
            # Generate test CSV
            generate_test_csv()
            wait_for_user()
            
        elif choice == 4:
            print("👋 Goodbye!")
            break
            
        else:
            print("❌ Invalid choice! Please select 1-4.")
            input("Press Enter to continue...")

if __name__ == "__main__":
    main()