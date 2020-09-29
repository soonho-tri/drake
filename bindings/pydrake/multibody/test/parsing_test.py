# -*- coding: utf-8 -*-

from pydrake.multibody.parsing import (
    Parser,
    PackageMap,
    LoadModelDirectives,
    ProcessModelDirectives,
)

import os
import unittest

from pydrake.common import FindResourceOrThrow
from pydrake.multibody.tree import (
    ModelInstanceIndex,
)
from pydrake.multibody.plant import (
    MultibodyPlant,
)


class TestParsing(unittest.TestCase):

    def test_package_map(self):
        dut = PackageMap()
        tmpdir = os.environ.get('TEST_TMPDIR')
        model = FindResourceOrThrow(
            "drake/examples/atlas/urdf/atlas_minimal_contact.urdf")

        # Simple coverage test for Add, Contains, size, GetPath.
        dut.Add("root", tmpdir)
        self.assertEqual(dut.size(), 1)
        self.assertTrue(dut.Contains("root"))
        self.assertEqual(dut.GetPath("root"), tmpdir)

        # Simple coverage test for Drake paths.
        dut.PopulateUpstreamToDrake(model)
        self.assertGreater(dut.size(), 1)

        # Simple coverage test for folder and environment.
        dut.PopulateFromEnvironment('TEST_TMPDIR')
        dut.PopulateFromFolder(tmpdir)

    def test_parser_file(self):
        """Calls every combination of arguments for the Parser methods which
        use a file_name (not contents) and inspects their return type.
        """
        sdf_file = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        urdf_file = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.urdf")
        for dut, file_name, model_name, result_dim in (
                (Parser.AddModelFromFile, sdf_file, None, int),
                (Parser.AddModelFromFile, sdf_file, "", int),
                (Parser.AddModelFromFile, sdf_file, "a", int),
                (Parser.AddModelFromFile, urdf_file, None, int),
                (Parser.AddModelFromFile, urdf_file, "", int),
                (Parser.AddModelFromFile, urdf_file, "a", int),
                (Parser.AddAllModelsFromFile, sdf_file, None, list),
                (Parser.AddAllModelsFromFile, urdf_file, None, list),
                ):
            plant = MultibodyPlant(time_step=0.01)
            parser = Parser(plant=plant)
            if model_name is None:
                result = dut(parser, file_name=file_name)
            else:
                result = dut(parser, file_name=file_name,
                             model_name=model_name)
            if result_dim is int:
                self.assertIsInstance(result, ModelInstanceIndex)
            else:
                assert result_dim is list
                self.assertIsInstance(result, list)
                self.assertIsInstance(result[0], ModelInstanceIndex)

    def test_parser_string(self):
        """Checks parsing from a string (not file_name)."""
        sdf_file = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        with open(sdf_file, "r") as f:
            sdf_contents = f.read()
        plant = MultibodyPlant(time_step=0.01)
        parser = Parser(plant=plant)
        result = parser.AddModelFromString(
            file_contents=sdf_contents, file_type="sdf")
        self.assertIsInstance(result, ModelInstanceIndex)

    def test_model_directives(self):
        model_dir = os.path.dirname(FindResourceOrThrow(
            "drake/multibody/parsing/test/"
            "process_model_directives_test/package.xml"))
        plant = MultibodyPlant(time_step=0.01)
        parser = Parser(plant=plant)
        parser.package_map().PopulateFromFolder(model_dir)
        directives_file = model_dir + "/add_scoped_top.yaml"
        directives = LoadModelDirectives(directives_file)
        added_models = ProcessModelDirectives(
            directives=directives, plant=plant, parser=parser)
        # Check for an instance.
        model_names = [model.model_name for model in added_models]
        self.assertIn("extra_model", model_names)
        plant.GetModelInstanceByName("extra_model")

    def test_model_directives_doc(self):
        """Check that the warning note in the docstring was added."""
        self.assertIn("Note:\n", ProcessModelDirectives.__doc__)
