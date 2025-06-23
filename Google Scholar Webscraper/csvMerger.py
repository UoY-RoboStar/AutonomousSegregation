import pandas as pd
import sys
from pathlib import Path

"""
Compare and merge two CSV files, removing duplicates.

Args:
    file1_path (str): Path to the first CSV file
    file2_path (str): Path to the second CSV file
    output_path (str): Path for the output merged CSV file
"""

def merge_csv_files(file1_path, file2_path, output_path):

    try:
        # Read both CSV files
        print(f"Reading {file1_path}...")
        df1 = pd.read_csv(file1_path)
        print(f"File 1 shape: {df1.shape}")
        
        print(f"Reading {file2_path}...")
        df2 = pd.read_csv(file2_path)
        print(f"File 2 shape: {df2.shape}")
        
        # Check if headers match
        if not df1.columns.equals(df2.columns):
            print("Warning: Column headers don't match exactly!")
            print(f"File 1 columns: {list(df1.columns)}")
            print(f"File 2 columns: {list(df2.columns)}")
            
            # Option to continue anyway
            response = input("Continue anyway? (y/n): ")
            if response.lower() != 'y':
                return
        
        # Combine the dataframes
        print("Combining files...")
        combined_df = pd.concat([df1, df2], ignore_index=True)
        print(f"Combined shape before removing duplicates: {combined_df.shape}")
        
        # Remove duplicates based on title column only
        print("Removing duplicates based on 'title' column...")
        merged_df = combined_df.drop_duplicates(subset=['Title'])
        print(f"Final shape after removing duplicates: {merged_df.shape}")
        
        # Select only the specified columns
        required_columns = ['Title', 'Snippet', 'Matches_Keywords', 'Condition1', 'Condition2', 'Condition3', 'Condition4']
        
        # Check if all required columns exist
        missing_columns = [col for col in required_columns if col not in merged_df.columns]
        if missing_columns:
            print(f"Warning: Missing columns: {missing_columns}")
            available_columns = [col for col in required_columns if col in merged_df.columns]
            print(f"Will save available columns: {available_columns}")
            merged_df = merged_df[available_columns]
        else:
            merged_df = merged_df[required_columns]
        
        # Calculate statistics
        total_rows = len(df1) + len(df2)
        duplicates_removed = total_rows - len(merged_df)
        print(f"Duplicates removed: {duplicates_removed}")
        
        # Save the merged file
        print(f"Saving merged file to {output_path}...")
        merged_df.to_csv(output_path, index=False)
        print("File saved successfully!")
        
        return merged_df
        
    except FileNotFoundError as e:
        print(f"Error: File not found - {e}")
    except pd.errors.EmptyDataError:
        print("Error: One of the CSV files is empty")
    except Exception as e:
        print(f"An error occurred: {e}")

"""
Main function to handle command line arguments or interactive input
"""
def main():
    if len(sys.argv) == 4:
        # Command line arguments provided
        file1_path = sys.argv[1]
        file2_path = sys.argv[2]
        output_path = sys.argv[3]
    else:
        # Interactive mode
        print("CSV File Merger - Remove Duplicates")
        print("=" * 40)
        
        file1_path = input("Enter path to first CSV file: ").strip()
        file2_path = input("Enter path to second CSV file: ").strip()
        output_path = input("Enter path for output file: ").strip()
        
        # Validate file paths
        if not Path(file1_path).exists():
            print(f"Error: {file1_path} does not exist")
            return
        if not Path(file2_path).exists():
            print(f"Error: {file2_path} does not exist")
            return
    
    # Perform the merge
    merge_csv_files(file1_path, file2_path, output_path)

if __name__ == "__main__":
    main()